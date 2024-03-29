# DEVELOPING #

These are just various development-related notes. <!-- Fun to play around with markdown formatting too! -->

## AdvantageKit Notes ##
In order to utilize AdvantageKit properly, different _Logger_ classes need be placed in between hardware I/O and user input to record data. For example, to log which option of a SmartDashboard _SendableChooser_ was chosen for an Autonomous routine, you would do something like this:
```java
SendableChooser<Command> chooser = new SendableChooser<>();
// Adding all the options for the SendableChooser goes here.
LoggedDashboardChooser<Command> autoChooser;
autoChooser = new LoggedDashboardChooser<>("Autonomous Choices", chooser);
```

## Limelight Notes ##
The static IP for the Limelight 2 is **10.58.29.18**.
The static IP for the Limelight 3G is **10.58.29.11**.
When physically connected by Ethernet, it can be found at at _limelight.local_, port **5801** for the configuration panel,
port **5800** for the camera stream.
> Note that the limelight relative to the center of the robot is roughly: 10 inches to the left, 20 inches up, and 2 inches forward.
> Nevermind, no longer true. Check again.

## Watch for changes upstream ##
- **Main template:** [Mechanical-Advantage/AdvantageKit/commits/main/example_projects/advanced_swerve_drive/src/main](https://github.com/Mechanical-Advantage/AdvantageKit/commits/main/example_projects/advanced_swerve_drive/src/main)
- **Limelight/AprilTag subsystem code:** [Hemlock5712/AdvantageKitSwerveTemplate/commits/main/src/main/java/frc/robot](https://github.com/Hemlock5712/AdvantageKitSwerveTemplate/commits/main/src/main/java/frc/robot)
- **LimelightHelper library:** [LimelightVision/limelightlib-wpijava/commits/main/](https://github.com/LimelightVision/limelightlib-wpijava/commits/main/)

## Useful links ##
- [Apriltag PDF Generator](https://tools.limelightvision.io/apriltag-generator)
