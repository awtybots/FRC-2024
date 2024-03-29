package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;

public class SpeakerShot {

  public static Command run(double SpeakerDistance, Arm arm) {
    return Commands.run(
        () -> {
          /* Physics calculation for the note:

          */

          // Position settings
          double ARMANGLE = ((0.345 * Math.PI * 2.0) - 0.18) / 2; // TODO Temporary

          arm.runTargetAngle(ARMANGLE);
        },
        arm);
  }
}
//  TODO check to add the `.until(
//             () -> {
//               return arm.getIsFinished();
//             });`
