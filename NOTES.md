# DEVELOPING #

## Useful links ##
- [Apriltag PDF Generator](https://tools.limelightvision.io/apriltag-generator)

## AdvantageKit Notes ##
In order to utilize AdvantageKit properly, different _Logger_ classes need be placed in between hardware I/O and user input to record data. For example, to log which option of a SmartDashboard _SendableChooser_ was chosen for an Autonomous routine, you would do something like this:
```java
SendableChooser<Command> chooser = new SendableChooser<>();
// Adding all the options for the SendableChooser goes here.
LoggedDashboardChooser<Command> autoChooser;
autoChooser = new LoggedDashboardChooser<>("Autonomous Choices", chooser);
```

## Limelight Notes ##
The static IP for the limelight is _10.58.29.18_.
When physically connected by Ethernet, it can be found at at _limelight.local_.
- Port 5801 for the configuration panel,
- Port 5800 for the camera stream.

## Watch for changes ##
- Main template: [Mechanical-Advantage/AdvantageKit/commits/main/example_projects/advanced_swerve_drive/src/main](https://github.com/Mechanical-Advantage/AdvantageKit/commits/main/example_projects/advanced_swerve_drive/src/main)
- Limelight code, AprilTag detection: [Hemlock5712/AdvantageKitSwerveTemplate/commits/main/src/main/java/frc/robot](https://github.com/Hemlock5712/AdvantageKitSwerveTemplate/commits/main/src/main/java/frc/robot)
- LimelightHelper library: [LimelightVision/limelightlib-wpijava/commits/main/](https://github.com/LimelightVision/limelightlib-wpijava/commits/main/)