// Copyright 2021-2024 FRC 6328, FRC 5829
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CombinedCommands.ShootNoteClose;
import frc.robot.commands.CombinedCommands.ShootNoteStart;
import frc.robot.commands.ControlCommands.ArmCommands;
import frc.robot.commands.ControlCommands.ClimberCommands;
import frc.robot.commands.ControlCommands.DriveCommands;
import frc.robot.commands.ControlCommands.IntakeShooterControls;
import frc.robot.commands.Positions.FloorPickup;
import frc.robot.commands.Positions.ShootClosePosition;
import frc.robot.commands.Positions.ShootFar;
import frc.robot.commands.Positions.ShootMedium;
import frc.robot.commands.Positions.SpeakerShot;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.commands.Positions.Upwards;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.intake.ColorSensorIO;
import frc.robot.subsystems.intake.ColorSensorIOV3;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
// import frc.robot.subsystems.sticks.Sticks;
// import frc.robot.subsystems.sticks.SticksIO;
// import frc.robot.subsystems.sticks.SticksIOSim;
// import frc.robot.subsystems.sticks.SticksIOSparkMax;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOSparkMax;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive sDrive;
  private final Flywheel sFlywheel;
  private final Intake sIntake;
  private final Arm sArm;
  //   private final ArmElevator sArmElevator;
  private final Wrist sWrist;
  private final Climber sClimber;
  //   private final Sticks sSticks;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs - this seems to be for auto testing so I won't be adding the others
  // See the Feedforward Characterizations
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);
  private final LoggedDashboardNumber armSpeedInput =
      new LoggedDashboardNumber("Arm Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.EnvironmentalConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        sDrive =
            new Drive(
                new GyroIONavX(false),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        sFlywheel = new Flywheel(new FlywheelIOSparkMax());
        sIntake = new Intake(new IntakeIOSparkMax() {}, new ColorSensorIOV3() {});
        sArm = new Arm(new ArmIOSparkMax() {});
        // sArmElevator = new ArmElevator(new ArmElevatorIOSparkMax() {});
        sWrist = new Wrist(new WristIOSparkMax() {});
        sClimber = new Climber(new ClimberIOSparkMax() {});
        // sSticks = new Sticks(new SticksIOSparkMax() {});

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // Note that most of these are broken and useless, and I don't think we have time to fix
        // them
        sDrive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        sFlywheel = new Flywheel(new FlywheelIOSim());
        sIntake = new Intake(new IntakeIOSim() {}, new ColorSensorIO() {});
        sArm = new Arm(new ArmIOSim() {});
        // sArmElevator = new ArmElevator(new ArmElevatorIOSim() {});
        sWrist = new Wrist(new WristIOSim() {});
        sClimber = new Climber(new ClimberIOSim() {});
        // sSticks = new Sticks(new SticksIOSim() {});

        break;

      default:
        // Replayed robot, disable IO implementations
        sDrive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        sFlywheel = new Flywheel(new FlywheelIO() {});
        sIntake = new Intake(new IntakeIO() {}, new ColorSensorIO() {});
        sArm = new Arm(new ArmIO() {});
        // sArmElevator = new ArmElevator(new ArmElevatorIO() {});
        sWrist = new Wrist(new WristIO() {});
        sClimber = new Climber(new ClimberIO() {});
        // sSticks = new Sticks(new SticksIO() {});

        break;
    }

    // Set up NamedCommands
    NamedCommands.registerCommand(
        "RunFlywheel",
        Commands.startEnd(
                () -> sFlywheel.runVelocity(flywheelSpeedInput.get()), sFlywheel::stop, sFlywheel)
            .withTimeout(5.0));

    NamedCommands.registerCommand(
        "RunIntake",
        Commands.startEnd(
                () -> sIntake.runPercentSpeed(Constants.IntakeConstants.percentPower),
                sIntake::stop,
                sIntake)
            .withTimeout(5.0));

    NamedCommands.registerCommand("FloorPickup", FloorPickup.run(sArm).withTimeout(2.0));

    NamedCommands.registerCommand(
        "SpeakerShot", SpeakerShot.run(1, sArm).withTimeout(5.0)); // TODO Replace SpeakerDistance

    NamedCommands.registerCommand(
        "ShootNoteStart", ShootNoteStart.run(sIntake, sArm, sFlywheel).withTimeout(5.0));

    NamedCommands.registerCommand(
        "ShootNoteClose", ShootNoteClose.run(sIntake, sArm, sFlywheel, sDrive).withTimeout(5.0));

    // Build SmartDashboard auto chooser
    if (!AutoBuilder.isConfigured()) {
      throw new RuntimeException(
          "AutoBuilder was not configured before attempting to build auto chooser");
    }

    SendableChooser<Command> chooser = new SendableChooser<>();

    // List of Autons to load on the RoboRIO. Do not change to the autogenerated version, as that
    // includes old versions of autons and so is unreliable.
    String[] autoNames =
        new String[] {
          // ! Meaning of auton naming scheme:
          // ! <Starting Position: L(Left Speaker), M(Middle Speaker), R(RightSpeaker)>

          // ! <First Row: Close,Far>
          // ! <Number Picked Up On First Row: 1,2,3>
          // ! <If M (Middle) Close3/Close2 or Far, which notes are picked up and order:
          // ! S1 (Shift to 1), S3 (Shift to 3), S13 (Shift to 1 then 3), S31 (Shift to 3 then 1),
          // ! and various Far ones. See below.>

          // ! <Second Row: Far>
          // ! <If M (Middle) Close3/Close2 or Far, which notes are picked up and order:
          // ! Same as before, but for FarS12 and the sort.>

          // Left group
          "L", // No movement, only a shot.
          "LClose", // Movement, no pickup.
          "LClose1", // Movement, one pickup and shot.
          "LClose2", // Movement, two pickups and shot.
          "LClose3", // Movement, three pickups and shot.
          "LClose1Far1", // Movement, one pickups and shot, far pickup and shot.
          "LClose2Far1", // Movement, two pickups and shot, far pickup and shot.
          "LClose3Far1", // Movement, three pickups and shot, far pickup and shot.

          // Middle group
          "M",
          "MClose",
          "MClose1",
          "MClose2S1",
          "MClose2S1Far1",
          "MClose2S3",
          "MClose3S31",
          "MClose3S31Far1",
          "MClose3S13",

          // Right group
          "R",
          "RClose",
          "RClose1",
          "RClose2",
          "RClose3",

          // Test Autos
          "SimpleStraight",
          "Spin180",
          "BiggerTestAuto"
        };

    chooser.setDefaultOption("None", Commands.none());
    List<PathPlannerAuto> options = new ArrayList<>();

    for (String autoName : autoNames) {
      PathPlannerAuto auto = new PathPlannerAuto(autoName);
      options.add(auto);
    }

    options.forEach(auto -> chooser.addOption(auto.getName(), auto));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", chooser);

    // Set up feedforward characterization
    // autoChooser.addOption(
    //     "Drive FF Characterization",
    //     new FeedForwardCharacterization(
    //         sDrive, sDrive::runCharacterizationVolts, sDrive::getCharacterizationVelocity));
    // autoChooser.addOption(
    //     "Flywheel FF Characterization",
    //     new FeedForwardCharacterization(
    //         sFlywheel, sFlywheel::runVolts, sFlywheel::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() { // Driver controller left and right joystick
    sDrive.setDefaultCommand(
        DriveCommands.joystickDrive(
            sDrive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // CommandScheduler.getInstance()
    //     .setDefaultCommand(
    //         sIntake,
    //         new ParallelCommandGroup(
    //             IntakeShooterControls.intakeShooterDrive(
    //                 sIntake,
    //                 sFlywheel,
    //                 () -> driverController.getLeftTriggerAxis(),
    //                 () -> driverController.getRightTriggerAxis(),
    //                 () -> false),
    //             IntakeShooterControls.intakeShooterDrive(
    //                 sIntake,
    //                 sFlywheel,
    //                 () -> operatorController.getLeftTriggerAxis(),
    //                 () -> operatorController.getRightTriggerAxis(),
    //                 () -> operatorController.leftBumper().getAsBoolean())));
    // # Alternate
    // sIntake.setDefaultCommand(
    //     IntakeShooterControls.intakeShooterDrive(
    //         sIntake,
    //         sFlywheel,
    //         () -> driverController.getLeftTriggerAxis(),
    //         () -> driverController.getRightTriggerAxis(),
    //         () -> false));
    sIntake.setDefaultCommand(
        IntakeShooterControls.intakeShooterDrive(
            sIntake,
            sFlywheel,
            () -> driverController.getLeftTriggerAxis(),
            () -> driverController.getRightTriggerAxis(),
            () -> driverController.leftBumper().getAsBoolean()));

    // # Alternate
    // driverController
    //     .rightTrigger()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () ->
    //                 sFlywheel.runVelocity(
    //                     -flywheelSpeedInput
    //                         .get()),
    //             sFlywheel::stop,
    //             sFlywheel));
    // driverController
    // .leftTrigger()
    // .whileTrue(
    //     Commands.startEnd(
    //         () ->
    //             sFlywheel.runVelocity(
    //                 -flywheelSpeedInput
    //                     .get()),
    //         sFlywheel::stop,
    //         sFlywheel));
    // Alternate #2
    // driverController
    //     .leftTrigger()
    //     .whileFalse(Commands.startEnd(() -> sFlywheel.runVelocity(0), sFlywheel::stop,
    // sFlywheel));

    // driverController
    // of
    //     .b()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> sIntake.runVelocity(Constants.IntakeConstants.velocity),
    //             sIntake::stop,
    //             sIntake));

    // driverController
    //     .b()
    //     .whileFalse(Commands.startEnd(() -> sIntake.runVelocity(0), sIntake::stop, sIntake));

    // driverController
    // of
    //     .x()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> sIntake.runVelocity(-Constants.IntakeConstants.velocity),
    //             sIntake::stop,
    //             sIntake));

    // driverController
    //     .b()
    //     .whileFalse(Commands.startEnd(() -> sIntake.runVelocity(0), sIntake::stop, sIntake));

    driverController // Reset Gyro
        .start()
        .whileTrue(Commands.run(() -> sDrive.resetRotation(), sDrive));

    // driverController // Slowmode
    //     .a()
    //     .toggleOnTrue(Commands.run(() -> sDrive.toggleSlowMode(), sDrive));

    // Climber Movement
    // operatorController.leftTrigger().whileTrue(ClimberCommands.buttonDrive(sClimber, () -> 1));
    // Commands.startEnd(
    //     () -> ClimberCommands.buttonDrive(sClimber, () -> 0.1), sClimber::stop, sClimber));
    // operatorController.rightTrigger().whileTrue(ClimberCommands.buttonDrive(sClimber, () -> -1));
    // Commands.startEnd(
    //     () -> ClimberCommands.buttonDrive(sClimber, () -> -0.1), sClimber::stop, sClimber));

    sClimber.setDefaultCommand(
        ClimberCommands.buttonDrive(
            sClimber, operatorController.leftBumper(), operatorController.rightBumper()));
    // Alternate for the above
    // operatorController
    //     .a()
    //     .whileTrue(
    //         Commands.startEnd(() -> sClimber.runTargetPosition(0), sClimber::stop, sClimber));
    // operatorController
    //     .b()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> sClimber.runTargetPosition(0.55), // !Testing numbers
    //             sClimber::stop,
    //             sClimber));

    // ## Operator controller configurations
    // Arm/wrist controls
    sArm.setDefaultCommand(
        ArmCommands.joystickDrive(sArm, operatorController.povUp(), operatorController.povDown()));

    // sWrist.setDefaultCommand(
    //     WristCommands.joystickDrive(sWrist, () -> operatorController.getLeftY()));

    // sArmElevator.setDefaultCommand(
    //     ArmElevatorCommands.triggerDrive(
    //         sArmElevator,
    //         () -> operatorController.getLeftTriggerAxis(),
    //         () -> operatorController.getRightTriggerAxis()));

    // operatorController
    //     .start()
    //     .whileTrue(Commands.startEnd(() -> sIntake.runFull(), sIntake::stop, sIntake));

    driverController.povDown().whileTrue(ShootFar.run(sArm));
    driverController.povRight().whileTrue(ShootMedium.run(sArm));
    driverController.povUp().whileTrue(ShootClosePosition.run(sArm));

    // run straight up position when y is pressed on operator. Using command Upwards
    operatorController.y().whileTrue(Upwards.run(sArm));

    // // run straight forwards position when x is pressed
    // operatorController.x().whileTrue(AmpShotPosition.run(sArm, sWrist));

    operatorController.a().whileTrue(FloorPickup.run(sArm));

    operatorController.b().whileTrue(StowPosition.run(sArm));

    // operatorController
    //     .x()
    //     .whileTrue(Commands.startEnd(() -> sSticks.runTargetAngle(0.0), sSticks::stop, sSticks));
    // operatorController
    //     .y()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> sSticks.runTargetAngle(0.5), sSticks::stop, sSticks)); // !Testing numbers
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
    // return Commands.none();
  }
}
