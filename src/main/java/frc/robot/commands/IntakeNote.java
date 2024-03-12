package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.armElevator.ArmElevator;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;

//Moves the note so that it is detected by the conveySensor but not shooterSensor
public class IntakeNote extends Command{ 


  private Intake intake;
  private Arm arm;


  public IntakeNote(Intake intake, Arm arm){
    this.intake = intake;
    this.arm = arm;
    addRequirements(intake, arm);
  }

  //Called once at the beginning
  @Override
  public void initialize() {



  }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    if (intake.getConveyerProximity()){
      intake.runPercentSpeed(0);
    }
    else{
      intake.runPercentSpeed(Constants.IntakeConstants.percentPower);

    }
  }
      // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


  }

  @Override
  public boolean isFinished() {
    return intake.getConveyerProximity();
  }

}
