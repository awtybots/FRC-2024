package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class StraightForwards {

  public static Command run(Arm arm, Wrist wrist) {
    return Commands.run(
        () -> {
          // Position preset settings
          double ARMANGLE = 0.111 * Math.PI * 2.0;
          double ARMELEVATORPOSITION = 0;
          double WRISTANGLE = 0;

          arm.runTargetAngle(ARMANGLE);
          wrist.runTargetAngle(WRISTANGLE);
        },
        arm,
        wrist);
  }
}
