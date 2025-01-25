# GRRBase
A template repository for Java-based robots in the FIRST Robotics Competition.

### Code Styling
All Java code follows the styling guide of [Prettier](https://prettier.io/). You can apply these rules via [Spotless](https://github.com/diffplug/spotless/tree/main) using the command `./gradlew spotlessApply`.

### Highlights

- [Swerve API](https://github.com/Greater-Rochester-Robotics/GRRBase/blob/main/src/main/java/org/team340/lib/swerve/SwerveAPI.java)

    Supports various hardware configurations, from vendors such as CTRE, REV, and Redux Robotics, with features such as high frequency odometry, a custom ratelimiter to improve driver control while also reducing wheel scrub, and built-in support for tuning the drivetrain's configuration live via NetworkTables.

- [Custom Dashboard](https://github.com/Greater-Rochester-Robotics/GRRBase/tree/main/dashboard)

    Includes an autonomous mode selection panel with animated previews of auto trajectories, and can be extended to add season-specific features.

- [Pre-defined Loggers](https://github.com/Greater-Rochester-Robotics/GRRBase/tree/main/src/main/java/org/team340/lib/logging)

    Utilizing Epilogue, an annotation-based logging framework, custom loggers for vendor and WPILib APIs are pre-defined to transparently log relevant data from the robot's hardware, without any extra setup required.

- [Various other utilities](https://github.com/Greater-Rochester-Robotics/GRRBase/tree/main/src/main/java/org/team340/lib/util)
