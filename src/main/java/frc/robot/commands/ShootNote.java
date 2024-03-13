package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;

public class ShootNote extends Command {

  private Intake intake;
  private Arm arm;
  private Flywheel flywheel;
  private boolean speedReached = false;
  private Long sensorsZeroTime = null;

  public ShootNote(Intake intake, Arm arm, Flywheel flywheel) {
    this.intake = intake;
    this.arm = arm;
    this.flywheel = flywheel;
    addRequirements(intake, flywheel);
  }

  @Override
  public void initialize() {
    sensorsZeroTime = null;
  }

  @Override
  public void execute() {
    double topFlywheelRPM = -flywheel.getVelocityRPMBottom();
    double targetRPM = Constants.FlywheelConstants.shootingVelocity * 0.9;
    flywheel.runVelocity(-Constants.FlywheelConstants.shootingVelocity);

    if (Math.abs(topFlywheelRPM) > Math.abs(targetRPM)) {
      intake.runPercentSpeed(1);
    }

    if (!intake.getConveyerProximity() && !intake.getShooterProximity()) {
      if (sensorsZeroTime == null) {
        sensorsZeroTime = System.currentTimeMillis();
      }
    } else {
      sensorsZeroTime = null;
    }
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.runVelocity(0);
    intake.runPercentSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return sensorsZeroTime != null && System.currentTimeMillis() - sensorsZeroTime >= 300;
  }
}
