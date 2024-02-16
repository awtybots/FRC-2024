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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveParts.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.armElevator.ArmElevator;
import frc.robot.subsystems.armElevator.ArmElevatorIO;
import frc.robot.subsystems.armElevator.ArmElevatorIOSim;
import frc.robot.subsystems.armElevator.ArmElevatorIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
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
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOSparkMax;
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
  private final ArmElevator sArmElevator;
  private final Wrist sWrist;

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
                new GyroIOPigeon2(false),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        sFlywheel = new Flywheel(new FlywheelIOSparkMax());
        sIntake = new Intake(new IntakeIOSparkMax() {});
        sArm = new Arm(new ArmIOSparkMax() {});
        sArmElevator = new ArmElevator(new ArmElevatorIOSparkMax() {});
        sWrist = new Wrist(new WristIOSparkMax() {});

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
        sIntake = new Intake(new IntakeIOSim() {});
        sArm = new Arm(new ArmIOSim() {});
        sArmElevator = new ArmElevator(new ArmElevatorIOSim() {});
        sWrist = new Wrist(new WristIOSim() {});

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
        sIntake = new Intake(new IntakeIO() {});
        sArm = new Arm(new ArmIO() {});
        sArmElevator = new ArmElevator(new ArmElevatorIO() {});
        sWrist = new Wrist(new WristIO() {});

        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> sFlywheel.runVelocity(flywheelSpeedInput.get()), sFlywheel::stop, sFlywheel)
            .withTimeout(5.0));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            sDrive, sDrive::runCharacterizationVolts, sDrive::getCharacterizationVelocity));
    autoChooser.addOption(
        "Flywheel FF Characterization",
        new FeedForwardCharacterization(
            sFlywheel, sFlywheel::runVolts, sFlywheel::getCharacterizationVelocity));

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
    sDrive.setDefaultCommand(
        DriveCommands.joystickDrive(
            sDrive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    driverController.x().onTrue(Commands.runOnce(sDrive::stopWithX, sDrive));
    // driverController
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     sDrive.setPose(
    //                         new Pose2d(sDrive.getPose().getTranslation(), new Rotation2d())),
    //                 sDrive)
    //             .ignoringDisable(true));
    driverController // TODO change to operatorController later
        .a()
        .whileTrue(
            Commands.startEnd(
                () -> sFlywheel.runVelocity(flywheelSpeedInput.get()), sFlywheel::stop, sFlywheel));
    driverController // TODO change to operatorController later
        .a()
        .whileFalse(Commands.startEnd(() -> sFlywheel.runVelocity(0), sFlywheel::stop, sFlywheel));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
