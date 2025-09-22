# Reefscape2025-340

FRC Team 340's code for the 2025 season, REEFSCAPE.

### Highlights

- **Full Field Localization**

    With 3 cameras used for AprilTag detection, the robot combines vision measurements with odometry using a Kalman filter to accurately track its position on the field throughout the entirety of a match.

- **Automated Reef Scoring**

    The robot's reef scoring sequence is fully automated, allowing the driver to press a single button to auto-drive to the selected reef pole and score.

    Additionally, the driver can strategically "stage" the robot's future motion by pointing their joystick in a desired exit direction, which the robot will follow immediately after a coral is scored, further reducing cycle times.

- **On-the-fly Motion Planning with Obstacle Avoidance**

    The robot utilizes a custom implementation of an artificial potential field to automatically servo to a specified pose, with the ability to avoid obstacles (i.e. field elements) in its path. This algorithm is used for all movement during the autonomous period, and for automated scoring in teleop.

- **Wildlife Conservation Program**

    The robot locks its superstructure from moving while it is within close proximity to the reef, ensuring our end effector (nicknamed the "Goose Neck") does not crash itself into a reef pole.

- **Anti-Beach Protocol**

    To prevent the robot from beaching itself on Algae, the robot will automatically drive in the opposite direction of the IMU's reported pitch/roll if they are above a set tolerance.

- **Sensory Driver Feedback**

    LEDs on the robot signal to the driver the current reef level selection as well as the robot's coral state, and haptic feedback from the driver's controller alerts them to when the robot has automatically switched to a new reef pole target, which is dependent on the robot's field position.

    While the robot is disabled, LEDs signal to the drive team information about AprilTag detection and autonomous mode selection, giving them the necessary feedback to effectively stage the robot pre-match.

### Robot Logs

Logs from matches during the 2025 season can be found [here](https://github.com/Greater-Rochester-Robotics/RobotLogs/tree/main/2025).

<br>

![Robot](robot.png)
