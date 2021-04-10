# octobotics Intern
Repo containing the codes as a part of the internship.
## Folder Structure
1. servo_control_msg Folder:
    * This folder must be added to <microros_ws>/firmware/mcu_ws/ directory.
    * This folder contains the code to create custom messages for servo motor control.
    * After copying the folder to the above directory, the following commands must be executed once to build the custom message headers.
        * ```
            ros2 run micro_ros_setup build_firmware.sh
            ``` 
        * Ensure that the commands
            ```
            source /opt/ros/foxy/setup.bash 
            source <microros_ws>/install/localsetup.bash
            ```
            have been executed before you executing the previous command.

2. servo_control folder:
    * This folder contains the source code for the Servo Motor Control.
    * This folder must be placed within the <microros_ws>/firmware/zephyr_apps/apps folder.
    * Only the basic structure for the codes has been uploaded now. (But these files compile and build with errors)
