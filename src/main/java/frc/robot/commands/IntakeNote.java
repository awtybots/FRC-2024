package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;
// Moves the note so that it is detected by the conveySensor but not shooterSensor

public class IntakeNote extends Command {

  private Intake intake;
  private Arm arm;
  private Flywheel flywheel;

  public IntakeNote(Intake intake, Arm arm, Flywheel flywheel) {
    this.intake = intake;
    this.arm = arm;
    this.flywheel = flywheel;
    addRequirements(intake, arm, flywheel);
  }

  // Called once at the beginning
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (intake.getConveyerProximity()) {
      intake.runPercentSpeed(0);
    } else {
      intake.runPercentSpeed(Constants.IntakeConstants.percentPower);
    }
  }
  // Run AdjustNote with a timeout of 4 seconds
  @Override
  public void end(boolean interrupted) {
    AdjustNote adjustNoteCommand = new AdjustNote(intake, arm, flywheel);
    adjustNoteCommand.withTimeout(1.5).schedule();
  }

  @Override
  public boolean isFinished() {
    return intake.getConveyerProximity();
  }
}
