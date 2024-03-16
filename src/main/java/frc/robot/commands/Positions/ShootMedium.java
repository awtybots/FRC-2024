package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;

public class ShootMedium { // one robot length away

  public static Command run(Arm arm) {
    return Commands.run(
            () -> {
              // Position preset settings
              double ARMANGLE = 0.933;

              arm.runTargetAngle(ARMANGLE);
            },
            arm)
        .until(
            () -> {
              return arm.getIsFinished();
            });
  }
}
