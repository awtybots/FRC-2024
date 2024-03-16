package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
<<<<<<< HEAD
import frc.robot.subsystems.intake.Intake;
=======
>>>>>>> dc6e0efa57445f0fd77e47415ec39bba8ffc12b0

public class FloorPickupCommand extends Command {

  private Arm arm;

<<<<<<< HEAD
  double ARMANGLE = 0.135;


=======
  double ARMANGLE = 0.075;
>>>>>>> dc6e0efa57445f0fd77e47415ec39bba8ffc12b0

  public FloorPickupCommand(Arm arm) {
    this.arm = arm;
  }

  // Called once at the beginning
  @Override
<<<<<<< HEAD
  public void initialize() {

  }
=======
  public void initialize() {}
>>>>>>> dc6e0efa57445f0fd77e47415ec39bba8ffc12b0

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.runTargetAngle(ARMANGLE);
<<<<<<< HEAD
    
=======
>>>>>>> dc6e0efa57445f0fd77e47415ec39bba8ffc12b0
  }

  // Called once the command ends or is interrupted.
  @Override
<<<<<<< HEAD
  public void end(boolean interrupted) {
  }
=======
  public void end(boolean interrupted) {}
>>>>>>> dc6e0efa57445f0fd77e47415ec39bba8ffc12b0

  @Override
  public boolean isFinished() {
    return arm.hasReachedDestination();
<<<<<<< HEAD

  }
}
=======
  }
}
>>>>>>> dc6e0efa57445f0fd77e47415ec39bba8ffc12b0
