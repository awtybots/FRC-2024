package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;

public class ShootFar { // two robots away from wall

  public static Command run(Arm arm) {
    return Commands.run(
        () -> {
          // Position preset settings
          double ARMANGLE = 0.7917;
          // double ARMELEVATORPOSITION = 0;
          // double WRISTANGLE = 0;

          arm.runTargetAngle(ARMANGLE);
          // armElevator.runTargetPosition(ARMELEVATORPOSITION);
          // wrist.runTargetAngle(WRISTANGLE);
        },
        arm);
  }
}
