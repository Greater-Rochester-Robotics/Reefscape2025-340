# Reefscape2025-340
FRC Team 340's code for the 2025 season, REEFSCAPE. 

### Highlights

- Full Field Localization

    With 3 cameras used for AprilTag detection, the robot combines vision measurements with odometry using a Kalman filter to accurately track its position on the field throughout the entirety of a match.

- Assisted Reef Alignment

    When the robot is being commanded to score, driver input is fused with an additional calculated assist to "push" the robot to center itself on a selected reef pole, all without fully removing control from the driver, allowing them to evade defense or scattered game pieces.

- Wildlife Conservation Program

    The robot locks its superstructure from moving while it is within close proximity to the reef, ensuring our end effector (nicknamed the "Goose Neck") does not crash itself into a reef pole.

- Anti-Beach Protocol

    To prevent the robot from beaching itself on Algae, the robot will automatically drive in the opposite direction of the IMU's reported pitch/roll if they are above a set tolerance.

### Robot Logs
Logs from matches during the 2025 season can be found [here](https://github.com/Greater-Rochester-Robotics/RobotLogs/tree/main/2025).

<br>

![Robot](robot.png)
