package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;

// Moves the note so that it is detected by the conveySensor but not shooterSensor
public class PreRunShooter extends Command {

  private Intake intake;
  private Arm arm;
  private Flywheel flywheel;

  public PreRunShooter(Intake intake, Arm arm, Flywheel flywheel) {
    this.intake = intake;
    this.arm = arm;
    this.flywheel = flywheel;
  }

  // Called once at the beginning
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    flywheel.runVelocity(-Constants.FlywheelConstants.shootingVelocity);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    intake.runPercentSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
