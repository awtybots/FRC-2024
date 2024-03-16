package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class FloorPickupCommand extends Command {

  private Arm arm;

  double ARMANGLE = 0.135;

  public FloorPickupCommand(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called once at the beginning
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.runTargetAngle(ARMANGLE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return arm.getIsFinished();
  }
}
