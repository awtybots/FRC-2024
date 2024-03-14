package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;

public class ShootNote extends Command {

  private Intake intake;
  private Flywheel flywheel;
  private Long sensorsZeroTime = null;
  private double AmpReductionFactor = 5; // amount to lower speed when doing amp
  private Arm arm;

  public ShootNote(Intake intake, Flywheel flywheel, Arm arm) {
    this.intake = intake;
    this.flywheel = flywheel;
    this.arm = arm;
    addRequirements(intake, flywheel);
  }

  @Override
  public void initialize() {
    sensorsZeroTime = null;
  }

  @Override
  public void execute() {
    double topFlywheelRPM = -flywheel.getVelocityRPMBottom();
    double targetRPM = Constants.FlywheelConstants.shootingVelocity;

    if (arm.getPosition() > 1) {
      targetRPM = Constants.FlywheelConstants.shootingVelocity / AmpReductionFactor;
    }
    flywheel.runVelocity(-targetRPM);

    if (Math.abs(topFlywheelRPM) > Math.abs(targetRPM * 0.9)) {
      intake.runPercentSpeed(0.5);
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
    flywheel.stop();

  }

  @Override
  public boolean isFinished() {
    return sensorsZeroTime != null && System.currentTimeMillis() - sensorsZeroTime >= 300;
  }
}
