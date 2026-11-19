1. Clone the ros2_kortex package and install dependencies:

    ```
    git clone https://github.com/Kinovarobotics/ros2_kortex.git src/third_party/kortex/ros2_kortex
    vcs import src/third_party/kortex --skip-existing --input src/third_party/kortex/ros2_kortex/ros2_kortex.jazzy.repos
    vcs import src/third_party/kortex --skip-existing --input src/third_party/kortex/ros2_kortex/ros2_kortex-not-released.jazzy.repos
    vcs import src/third_party/kortex --skip-existing --input src/third_party/kortex/ros2_kortex/simulation.jazzy.repos
    ```

2. Clone the ros2_kortex_vision package and install:

    ```
    git clone https://github.com/Kinovarobotics/ros2_kortex_vision.git src/third_party/kortex/ros2_kortex_vision
    ```

3. Build with colcon 

    ```rosdep install --ignore-src --from-paths src -y -r
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    ```


ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
points: [
    { positions: [0.00, -0.47, 3.14, -2.30, 0.00, -1.25, 1.57], time_from_start: { sec: 5 } },
]
}" -1

ros2 control switch_controllers --activate twist_controller --deactivate joint_trajectory_controller
