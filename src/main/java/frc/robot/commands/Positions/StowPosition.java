package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
// import frc.robot.subsystems.wrist.Wrist;

public class StowPosition {

  public static Command run(Arm arm) {
    return Commands.run(
        () -> {
          // Position preset settings
          double ARMANGLE = 0.35;
          // double WRISTANGLE = 0;

          arm.runTargetAngle(ARMANGLE);
          // wrist.runTargetAngle(WRISTANGLE);
        },
        arm);
  }
}
