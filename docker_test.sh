
#!/bin/bash

# Build and run the docker containers
if [ "$1" == "1" ]; then
    # Clean macOS metadata files before starting
    echo "Cleaning macOS metadata files..."
    find . -name "._*" -type f -delete
    find . -name ".DS_Store" -type f -delete
    
    docker-compose up -d px4-ros2
    docker cp src px4_ros2_dev:/home/workspace/
    docker cp launch px4_ros2_dev:/home/workspace/
    docker cp config px4_ros2_dev:/home/workspace/
    docker cp package.xml px4_ros2_dev:/home/workspace/
    docker cp CMakeLists.txt px4_ros2_dev:/home/workspace/
    docker cp CMakeLists_simple.txt px4_ros2_dev:/home/workspace/
    
    # Clean metadata files inside container and build
    docker exec -i px4_ros2_dev bash -c "cd /home/workspace && rm -rf install build log && find . -name '._*' -delete && cp CMakeLists_simple.txt CMakeLists.txt && source /opt/ros/humble/setup.bash && colcon build"
    
    # Clean up metadata files after build
    docker exec -i px4_ros2_dev bash -c "cd /home/workspace && find install -name '._*' -exec rm -f {} \; 2>/dev/null || true"

# Run the tests
elif [ "$1" == "2" ]; then
    docker exec -i px4_ros2_dev bash -c "cd /home/workspace && . install/setup.bash && ros2 launch offboard_control simple_test.launch.py"

# Stop and remove the containers
elif [ "$1" == "3" ]; then
    docker-compose stop px4-ros2 && docker-compose rm -f px4-ros2

else
    echo "Usage: ./docker_test.sh [1|2|3]"
    echo "1: Build and run the docker containers"
    echo "2: Run the tests"
    echo "3: Stop and remove the containers"
fi
