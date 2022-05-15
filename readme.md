PATH FOLLOW ALGORITHM IN ROS2 GALACTIC WITH CPP FOR FLUID DEV


- UNDER THE FOLDER "VIDEO" YOU CAN FIND THE RECORDING OF RVIZ WHILE EXECUTING THE ALGORITHM.

- MAIN ASSUMPTION:

    - ALL 6 DOF CAN BE ACTUATES ON THE ROBOT LINK.

    - THE ROBOT LINK IS REPRESENTED AS A FRAME (COORD AXIS)

- STRUCTURE:

    - PARAMETER UNDER THE FOLDER NAME "CONFIG"

        - ALLOWS TO DEFINE INITIAL POSE AND PATH POINTS.

    - LAUNCH UNDER THE "LAUNCH" FOLDER.

        - ALLOWS TO SPAWN THE ALGORITHM AND THE VISUALIZATION ON RVIZ2

    - TEST FOLDER TO SHOW HOW TO STRUCTURE UNIT TESTS USING GTEST IN A ROS2 PACKAGE

        - FURTHER TEST NEEDED TO BE DEFINED

    - INCLUDE FOLDER IS NOT BEING USED. SHOULD BE USED TO SEPARATE THE CODE IMPLENTATION FROM THE STRUCTURE.

- ROS2 STRUCTURE:

    - COMPOSABLE NODE CONTAINERS

        - robot_link_init TO START THE PATH PUBLISH AND SET FIRST TRANFROM OF ROBOT LINK
        - robot_link_path_follow_container DEFINES THE CONTROL OF THE ROBOT AND EXECUTES THE COMMANDS TAKEN INTO ACCOUNT THE MODEL OF THE ROBOT

    - STATIC TRANSFORMS

        - THE PATH IS DEFINED IN A YAML FILE AND IT IS PUBLISHED AT FIRST AS A SERIES OF STATIC TRANFORMS WHICH ARE DEFINED IN ITS PREVIOUS POINT FRAME.


- BEFORE EXECUTION
    - PLACE PACKAGE UNDER A ROS2 WORKSPACE SRC FOLDER
    - BUILD WITH THE FOLLOWING COMMAND: colcon build --allow-overriding path_follow --packages-select path_follow
    - SOURCE THE PACKAGE
    - MODIFY "config/params.yaml" as needed to change path to follow and initial pose of robot link
    - LAUNCH THE ALGORITHM BY USING THE PREPARED LAUNCH FILE: ros2 launch path_follow path.launch.py