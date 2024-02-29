package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.armElevator.ArmElevator;
import frc.robot.subsystems.wrist.Wrist;

public class ShootFar { // two robots away from wall

  public static Command run(Arm arm, ArmElevator armElevator, Wrist wrist) {
    return Commands.run(
        () -> {
          // Position preset settings
          double ARMANGLE = 0.7917;
          double ARMELEVATORPOSITION = 0;
          double WRISTANGLE = 0;

          arm.runTargetAngle(ARMANGLE);
          armElevator.runTargetPosition(ARMELEVATORPOSITION);
          wrist.runTargetAngle(WRISTANGLE);
        },
        arm,
        armElevator,
        wrist);
  }
}
