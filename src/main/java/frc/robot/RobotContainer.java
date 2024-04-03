// Copyright 2016-2024 FRC 5829, FRC 6328
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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ControlCommands.ArmCommands;
import frc.robot.commands.ControlCommands.DriveCommands;
import frc.robot.commands.ControlCommands.IntakeShooterControls;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.IntakeNoteAndAlign;
import frc.robot.commands.Positions.AmpShot;
import frc.robot.commands.Positions.FloorPickup;
import frc.robot.commands.Positions.ShootClose;
import frc.robot.commands.Positions.ShootFar;
import frc.robot.commands.Positions.ShootMedium;
import frc.robot.commands.Positions.SpeakerShot;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.commands.PreRunShooter;
import frc.robot.commands.ShootNote;
import frc.robot.subsystems.LedSubsystem;
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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.ProximitySensorIOV3;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIO;
import frc.robot.subsystems.vision.AprilTagVisionIOLimelight;
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
  private final Climber sClimber;
  private final LedSubsystem ledSubsystem;
  private AprilTagVision aprilTagVision;

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
                new GyroIONavX(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        sFlywheel = new Flywheel(new FlywheelIOSparkMax());
        sIntake = new Intake(new IntakeIOSparkMax() {}, new ProximitySensorIOV3() {});
        sArm = new Arm(new ArmIOSparkMax() {});
        sClimber = new Climber(new ClimberIOSparkMax() {});
        aprilTagVision = new AprilTagVision(new AprilTagVisionIOLimelight("limelight"));

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
        sIntake = new Intake(new IntakeIOSim() {}, new ProximitySensorIOV3() {});
        sArm = new Arm(new ArmIOSim() {});
        sClimber = new Climber(new ClimberIOSim() {});
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});

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
        sIntake = new Intake(new IntakeIO() {}, new ProximitySensorIOV3() {});
        sArm = new Arm(new ArmIO() {});
        sClimber = new Climber(new ClimberIO() {});
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});

        break;
    }

    ledSubsystem = new LedSubsystem(0, 80, sIntake);

    aprilTagVision.setDataInterfaces(sDrive::addVisionData);

    // SmartDashboard.putData(CommandScheduler.getInstance()); //! Command check (testing)

    // Build SmartDashboard auto chooser
    if (!AutoBuilder.isConfigured()) {
      throw new RuntimeException(
          "AutoBuilder was not configured before attempting to build auto chooser and named commands");
    }

    // Set up NamedCommands

    NamedCommands.registerCommand("IntakeNote", new IntakeNote(sIntake).withTimeout(4.0));

    NamedCommands.registerCommand(
        "ShootNote", new ShootNote(sIntake, sFlywheel, sArm).withTimeout(3.0));

    NamedCommands.registerCommand("FloorPickupPosition", FloorPickup.run(sArm).withTimeout(2.0));

    NamedCommands.registerCommand("ShootClosePosition", ShootClose.run(sArm).withTimeout(2.0));

    NamedCommands.registerCommand("ShootMediumPosition", ShootMedium.run(sArm).withTimeout(2.0));

    NamedCommands.registerCommand("ShootFarPosition", ShootFar.run(sArm).withTimeout(2.0));

    // NamedCommands.registerCommand(
    //     "FloorPickupPosition", new FloorPickupCommand(sArm).withTimeout(2.0));

    // NamedCommands.registerCommand(
    //     "ShootClosePosition", new ShootCloseCommand(sArm).withTimeout(2.0));

    // NamedCommands.registerCommand(
    //     "ShootMediumPosition", new ShootMediumCommand(sArm).withTimeout(2.0));

    // NamedCommands.registerCommand("ShootFarPosition", new
    // ShootFarCommand(sArm).withTimeout(2.0));

    NamedCommands.registerCommand(
        "PreRunShooter", new PreRunShooter(sFlywheel, sIntake).withTimeout(4.0));

    // Groups of the above
    NamedCommands.registerCommand(
        "StartGroup",
        new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                ShootClose.run(sArm).withTimeout(2),
                new PreRunShooter(sFlywheel, sIntake).withTimeout(4.0)),
            new ShootNote(sIntake, sFlywheel, sArm).withTimeout(3.0)));

    NamedCommands.registerCommand( // The name is inaccurate
        "ShootMediumGroup", new ShootNote(sIntake, sFlywheel, sArm).withTimeout(3.0));

    // In testing
    NamedCommands.registerCommand(
        "SpeakerShot", SpeakerShot.run(1, sArm).withTimeout(5.0)); // TODO Replace SpeakerDistance

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

          // Right Alternate Group
          "RCloseOut",
          "RFar1S5",

          // Test Autos
          "SimpleStraight",
          "Spin180",
          "BiggerTestAuto",
          "BackTestAuto"
        };

    chooser.setDefaultOption("None", Commands.none());
    List<PathPlannerAuto> options = new ArrayList<>();

    for (String autoName : autoNames) {
      PathPlannerAuto auto = new PathPlannerAuto(autoName);
      options.add(auto);
    }

    options.forEach(auto -> chooser.addOption(auto.getName(), auto));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", chooser);

    // Set up SysId routines
    // Drive subsystem
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        sDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        sDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", sDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", sDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Flywheel subsystem
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        sFlywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        sFlywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)",
        sFlywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)",
        sFlywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Climber subsystem
    autoChooser.addOption(
        "Climber SysId (Quasistatic Forward)",
        sClimber.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Climber SysId (Quasistatic Reverse)",
        sClimber.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Climber SysId (Dynamic Forward)", sClimber.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Climber SysId (Dynamic Reverse)", sClimber.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Arm subsystem
    autoChooser.addOption(
        "Arm SysId (Quasistatic Forward)", sArm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Arm SysId (Quasistatic Reverse)", sArm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Arm SysId (Dynamic Forward)", sArm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Arm SysId (Dynamic Reverse)", sArm.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Intake subsystem
    autoChooser.addOption(
        "Intake SysId (Quasistatic Forward)",
        sIntake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Intake SysId (Quasistatic Reverse)",
        sIntake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Intake SysId (Dynamic Forward)", sIntake.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Intake SysId (Dynamic Reverse)", sIntake.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // # Driver controller configuration

    sDrive.setDefaultCommand(
        DriveCommands.joystickDrive(
            sDrive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            false));

    driverController
        .x()
        .whileTrue(
            DriveCommands.joystickDrive(
                sDrive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                true));

    driverController
        .start()
        .whileTrue(Commands.startEnd(() -> sDrive.resetRotation(), sDrive::stop, sDrive));

    operatorController
        .start()
        .whileTrue(Commands.startEnd(() -> sIntake.runFull(), sIntake::stop, sIntake));

    driverController.rightBumper().whileTrue(Commands.runOnce(() -> sDrive.toggleSlowMode()));

    // ! TEST <
    driverController.a().whileTrue(sDrive.getZeroAuton());
    // !TEST >

    // # Operator controller configuration

    // Arm Controls (Controlled by right stick Y-axis)
    sArm.setDefaultCommand(ArmCommands.joystickDrive(sArm, () -> -operatorController.getRightY()));

    // Arm Positions
    operatorController.povDown().whileTrue(ShootFar.run(sArm));
    operatorController.povRight().whileTrue(ShootMedium.run(sArm));
    operatorController.povUp().whileTrue(ShootClose.run(sArm));

    // operatorController.y().whileTrue(Upwards.run(sArm)); //Note that Y is taken
    operatorController.x().whileTrue(AmpShot.run(sArm));
    operatorController.a().whileTrue(FloorPickup.run(sArm));
    operatorController.b().whileTrue(StowPosition.run(sArm));

    // Intake Controls (Three ways: #1 right trigger, which goes backwards, #2 “Y”, full power, and
    // #3 is left trigger, which has sensor logic)
    sIntake.setDefaultCommand(
        IntakeShooterControls.intakeShooterDefaultCommand(
            sIntake, () -> operatorController.getRightTriggerAxis()));

    operatorController
        .y()
        .whileTrue(Commands.startEnd(() -> sIntake.runPercentSpeed(-1), sIntake::stop, sIntake));

    operatorController.leftTrigger().whileTrue(new IntakeNoteAndAlign(sIntake));
    // operatorController.leftTrigger().whileTrue(new IntakeNote(sIntake));

    // Flywheel commands
    operatorController.rightBumper().whileTrue(new ShootNote(sIntake, sFlywheel, sArm));
    operatorController.leftBumper().whileTrue(new PreRunShooter(sFlywheel, sIntake));
    sFlywheel.setDefaultCommand(
        new PreRunShooter(sFlywheel, true, sIntake)); // Runs the flywheel slowly at all times

    // Climber controls (The first one is 90% probably the one that works.)
    // sClimber.setDefaultCommand(
    //     ClimberCommands.buttonDrive(
    //         sClimber, operatorController.leftBumper(), operatorController.rightBumper()));

    //     operatorController
    // .a()
    // .whileTrue(
    //     Commands.startEnd(() -> sClimber.runTargetPosition(0), sClimber::stop, sClimber));

    // operatorController
    //     .b()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> sClimber.runTargetPosition(0.55), // !Testing numbers
    //             sClimber::stop,
    //             sClimber));

    // driverController
    //     .povUp()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> ClimberCommands.buttonDrive(sClimber, () -> 1), sClimber::stop, sClimber));
    // driverController
    //     .povDown()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> ClimberCommands.buttonDrive(sClimber, () -> -1), sClimber::stop,
    // sClimber));
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
