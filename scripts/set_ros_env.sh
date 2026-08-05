#/usr/bin/env sh

# Ensure conda libs take precedence over system libs
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH

ROS_ENV_FILE="scripts/personal_ros_env.sh"

if [ -f "$ROS_ENV_FILE" ]; then
    echo "$ROS_ENV_FILE already exists. Sourcing it..."
    . "$ROS_ENV_FILE"
else
    if [ "$ROS_DISTRO" = "jazzy" ] || [ "$ROS_DISTRO" = "humble" ]; then
        echo "$ROS_ENV_FILE not found. Using default environment variables..."
        # ${VAR:-default} so an explicit ROS_DOMAIN_ID from the caller wins. The IRiS FR3 rig
        # runs on 77: domain 100 is shared with long-running ROS 2 Jazzy nodes on that network,
        # and cross-distro discovery data makes this Humble stack's CycloneDDS log
        # 'string data is not null-terminated' / 'invalid data size' (serdata.cpp:384)
        # continuously. Reproducible with a bare node and zero subscriptions.
        export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-100}"
        export ROS_LOCALHOST_ONLY=0
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    fi
fi

ros2 daemon stop
ros2 daemon start
