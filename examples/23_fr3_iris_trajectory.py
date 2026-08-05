#!/usr/bin/env python
"""Run a trajectory on the IRiS FR3, headless, against the iris-panda-ros2 bring-up.

Companion to 01_figure_eight.py, for the FR3 on the IRiS rig. Differences that matter:

  * 01_figure_eight.py uses make_robot("fr3"), whose target_frame is fr3_hand_tcp. The rig is
    usually brought up bare-flange (attachment:=none), where the controllers servo fr3_link8,
    0.1034 m away. Default here is the fr3_flange preset.
  * It ends in plt.show(), which blocks forever over ssh with no DISPLAY. This writes a PNG.
  * It homes unconditionally as its FIRST motion, which from an arbitrary pose is a large
    unguarded sweep. Here homing is opt-in (--home), and --mode nudge exists to exercise the
    whole path with a few centimetres of travel first.
  * It loads a cartesian-impedance parameter config from config/control/. This does not, so it
    runs against whatever iris_bringup's controllers_fr3*.yaml already configured.

DISTRO: the rig's FR3 runs ROS 2 HUMBLE (franka_ros2 v1.0.0 + libfranka 0.13.6 - the only
combination speaking FCI server version 7, which is what robot system 5.6.0 offers). Run this
from crisp_py's `humble` environment:

    pixi run -e humble python examples/23_fr3_iris_trajectory.py

Do NOT use the `jazzy` environment against this rig: scripts/set_ros_env.sh ends with
`ros2 daemon stop; ros2 daemon start`, so a jazzy pixi run restarts the ros2 daemon as Jazzy,
and it then serves unusable discovery data to every Humble CLI on the same ROS_DOMAIN_ID.

Bring the robot up first, on the machine wired to it:

    pixi run -e humble-fr3 ros2 launch iris_bringup iris_fr3_humble.launch.py \\
        robot_ip:=192.168.50.231 attachment:=none

THIS SCRIPT COMMANDS REAL MOTION. Keep the enabling device in hand.
"""

import argparse
import sys
import time

import matplotlib

matplotlib.use("Agg")  # before pyplot: no DISPLAY over ssh

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from crisp_py.robot import make_robot  # noqa: E402


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mode", choices=["nudge", "figure8"], default="nudge",
                   help="nudge: small relative move, the smallest thing that exercises the full "
                        "path. figure8: the y-z figure eight. Default: nudge.")
    p.add_argument("--robot-config", default="fr3_flange",
                   help="crisp_py robot preset. fr3_flange (bare flange, fr3_link8), "
                        "fr3 (Franka Hand, fr3_hand_tcp), fr3_umi (camera frame). "
                        "MUST match the launch's attachment. Default: fr3_flange")
    p.add_argument("--home", action="store_true",
                   help="Home before moving. This is a LARGE sweep from an arbitrary pose - "
                        "clear the workspace first. Off by default.")
    p.add_argument("--plot", default="/tmp/fr3_iris_trajectory.png", help="PNG output path.")
    p.add_argument("--nudge-delta", type=float, default=0.05, help="Nudge distance [m].")
    p.add_argument("--radius", type=float, default=0.2, help="figure8 radius [m].")
    p.add_argument("--center", type=float, nargs=3, default=[0.4, 0.0, 0.4], help="figure8 centre.")
    p.add_argument("--max-time", type=float, default=8.0, help="figure8 duration [s].")
    p.add_argument("--speed", type=float, default=0.15, help="move_to speed [m/s].")
    p.add_argument("--ctrl-freq", type=float, default=50.0, help="command rate [Hz].")
    p.add_argument("--yes", action="store_true", help="Skip the confirmation prompt.")
    return p.parse_args()


def confirm(args, robot):
    print(f"\n  preset      : {args.robot_config}  (target_frame {robot.config.target_frame})")
    print(f"  mode        : {args.mode}")
    print(f"  home first  : {args.home}")
    print(f"  current pose: {np.round(robot.end_effector_pose.position, 4).tolist()}")
    if args.mode == "figure8":
        print(f"  will move to: {args.center}, then a {args.radius} m figure eight")
    else:
        print(f"  will move   : {args.nudge_delta} m in +z, then back")
    if args.yes:
        return True
    reply = input("\nThis commands REAL MOTION. Type 'go' to continue: ").strip().lower()
    return reply == "go"


def main():
    args = parse_args()
    rclpy.init()
    robot = make_robot(config_name=args.robot_config)
    robot.wait_until_ready(timeout=30.0)

    if robot.config.target_frame == "fr3_hand_tcp" and args.robot_config == "fr3":
        print("NOTE: preset 'fr3' assumes a Franka Hand. If you launched attachment:=none, "
              "every coordinate below is 0.1034 m off. Use --robot-config fr3_flange.",
              file=sys.stderr)

    print(f"joints: {np.round(robot.joint_values, 4).tolist()}")
    if not confirm(args, robot):
        print("aborted.")
        robot.shutdown(); rclpy.try_shutdown(); return 1

    if args.home:
        print("homing (large motion)...")
        robot.home(blocking=True)

    robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")
    time.sleep(0.5)

    start = robot.end_effector_pose.copy()
    ee, tgt, ts = [], [], []

    if args.mode == "nudge":
        goal = start.copy()
        goal.position = start.position + np.array([0.0, 0.0, args.nudge_delta])
        print(f"nudging +{args.nudge_delta} m in z ...")
        robot.move_to(position=goal.position, speed=args.speed)
        time.sleep(1.0)
        print(f"reached : {np.round(robot.end_effector_pose.position, 4).tolist()}")
        print("returning ...")
        robot.move_to(position=start.position, speed=args.speed)
        time.sleep(1.0)
        print(f"back at : {np.round(robot.end_effector_pose.position, 4).tolist()}")
        err = float(np.linalg.norm(robot.end_effector_pose.position - start.position))
        print(f"return error: {err * 1000:.1f} mm")
    else:
        center = np.array(args.center)
        print(f"moving to centre {center.tolist()} ...")
        robot.move_to(position=center, speed=args.speed)

        print("drawing figure eight ...")
        target = robot.end_effector_pose.copy()
        rate = robot.node.create_rate(args.ctrl_freq)
        t = 0.0
        while t < args.max_time + 1.0:
            if t < args.max_time:
                y = args.radius * np.sin(2 * np.pi * 0.25 * t) + center[1]
                z = args.radius * np.sin(2 * np.pi * 0.125 * t) + center[2]
                target.position = np.array([center[0], y, z])
                robot.set_target(pose=target)
            rate.sleep()
            ee.append(robot.end_effector_pose.copy())
            tgt.append(robot._target_pose.copy())
            ts.append(t)
            t += 1.0 / args.ctrl_freq

        y_ee = [p.position[1] for p in ee]
        z_ee = [p.position[2] for p in ee]
        y_t = [p.position[1] for p in tgt]
        z_t = [p.position[2] for p in tgt]
        err = np.linalg.norm(np.array([y_ee, z_ee]) - np.array([y_t, z_t]), axis=0)
        print(f"tracking error: mean {err.mean()*1000:.1f} mm, max {err.max()*1000:.1f} mm")

        fig, ax = plt.subplots(1, 2, figsize=(10, 5))
        ax[0].plot(y_ee, z_ee, label="current")
        ax[0].plot(y_t, z_t, "--", label="target")
        ax[0].set_xlabel("y [m]"); ax[0].set_ylabel("z [m]"); ax[0].set_aspect("equal")
        ax[1].plot(ts, z_ee, label="current")
        ax[1].plot(ts, z_t, "--", label="target")
        ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("z [m]"); ax[1].legend()
        for a in ax:
            a.grid()
        fig.suptitle(f"FR3 figure eight - {args.robot_config} ({robot.config.target_frame})")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=120)
        print(f"plot written to {args.plot}")

    if args.home:
        print("homing back ...")
        robot.home(blocking=True)

    robot.shutdown()
    rclpy.try_shutdown()
    print("done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
