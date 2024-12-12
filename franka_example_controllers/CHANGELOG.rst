^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package franka_example_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Topic: Add franka_ros2 repo to ROS2 Distribution Index
    BREAKING_CHANGE: joint_trajectory_controller, use upstream ros2_controllers
    build: required package.xml files and CMakeLists.txt
    build: Use a single Dockerfile for both ./.devcontainer and ./ (soft-link).
    doc: README.md
    doc: add CHANGELOG.rst
* feat: examples start from current system state
* fix: resolve acceleration discontinuities in example controllers
* fix: unit tests and removed time check
* fix: fix example controller joint position example acceleration discontinuity
* fix: joint_impedance_with_ik works with and without gripper
* hotfix: fix example controller joint position example acceleration discontinuity
* refactor: define time_out when waiting for requests in one constant
* Contributors: Baris Yazici, Jack Smith, Marius Winkelmeier