package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;

// Moves the note so that it is detected by the conveySensor but not shooterSensor
public class AdjustNote extends Command {

  private Intake intake;
  private Arm arm;
  private Flywheel flywheel;
  private int phase =
      1; // 1 is moving note to shootersensor, 2 is moving note away from shooterSensor, 3 = done
  private double forwardsIntakeSpeed = 0.05;
  private double backwardsIntakeSpeed = 0.07;

  public AdjustNote(Intake intake, Arm arm, Flywheel flywheel) {
    this.intake = intake;
    this.arm = arm;
    this.flywheel = flywheel;
    addRequirements(intake, arm, flywheel);
  }

  // Called once at the beginning
  @Override
  public void initialize() {
    phase = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (phase == 1) { // go to shooter sensor untill detected
      intake.runPercentSpeed(forwardsIntakeSpeed);
      if (intake.getShooterProximity()) {
        phase = 2;
      }

    } else if (phase == 2) { // go backwards untill not detected
      intake.runPercentSpeed(-backwardsIntakeSpeed);
      if (!intake.getShooterProximity()) {
        phase = 3;
      }
    }
    if (phase == 3) {
      intake.runPercentSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    phase = 1;
  }

  @Override
  public boolean isFinished() {
    return phase == 3;
  }
}
