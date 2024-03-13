package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;

public class ShootNote extends Command {

  private Intake intake;
  private Flywheel flywheel;
  private Long sensorsZeroTime = null;

  public ShootNote(Intake intake, Flywheel flywheel) {
    this.intake = intake;
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
    double targetRPM = Constants.FlywheelConstants.shootingVelocity * 0.8;
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
