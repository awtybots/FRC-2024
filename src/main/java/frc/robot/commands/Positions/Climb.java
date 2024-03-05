package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;

public class Climb {

  public static Command run(Arm arm) {
    return Commands.run(
        () -> {
          // Position preset settings
          double ARMANGLE = 2.53;

          arm.runTargetAngle(ARMANGLE);
        },
        arm);
  }
}
