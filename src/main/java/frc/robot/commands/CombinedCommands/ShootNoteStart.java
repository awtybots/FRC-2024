package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ControlCommands.IntakeShooterControls;
import frc.robot.commands.Positions.ShootClosePosition;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.arm.Arm;
// import frc.robot.subsystems.armElevator.ArmElevator;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;

public class ShootNoteStart {
  // Note to self: inspiration for future (and if this doesn't work lol):
  // https://github.com/Hemlock5712/AdvantageKitSwerveTemplate/tree/main/src/main/java/frc/robot/commands

  public ShootNoteStart() {}

  public static Command run(Intake sIntake, Arm sArm, Flywheel sFlywheel) {
    return Commands.run(
        () -> {
          ShootClosePosition.run(sArm);
          IntakeShooterControls.intakeShooterDrive(sIntake, sFlywheel, () -> 0, () -> 1, () -> true)
              .withTimeout(3);
          StowPosition.run(sArm);
        },
        sIntake,
        sArm,
        sFlywheel);
  }
}
