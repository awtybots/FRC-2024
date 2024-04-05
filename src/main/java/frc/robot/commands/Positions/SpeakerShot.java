// Copyright 2016-2024 FRC 5829
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands.Positions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.Optional;

public class SpeakerShot {

  public static Command run(Arm arm, Drive drive) {
    return Commands.run(
        () -> {
          Pose2d currentPose = drive.getPose();
          Optional<Double> speakerDistance = Optional.empty();
          try {
            speakerDistance =
                Optional.of(
                    Math.sqrt(
                        Math.pow(
                                currentPose.getX()
                                    - AllianceFlipUtil.apply(
                                            FieldConstants.Speaker.centerSpeakerOpening
                                                .getTranslation())
                                        .getX(),
                                2)
                            + Math.pow(
                                currentPose.getY()
                                    - AllianceFlipUtil.apply(
                                            FieldConstants.Speaker.centerSpeakerOpening
                                                .getTranslation())
                                        .getY(),
                                2)));
          } catch (Exception e) {
          }

          arm.runTargetAngle(getRequiredAngle(speakerDistance));
        },
        arm);
  }

  /**
   * Gets the arm angle required for the note to score into the speaker. Calculates this using a
   * polynomial regression of 5 known points.
   */
  public static Optional<Double> getRequiredAngle(Optional<Double> speakerDistance) {
    // Numbers calculated using Desmos. Note that the range is between 1.753 (upwards) and 0.09
    // (floor pickup) at the moment. Polynomial form ax^2 + bx + c.

    if (-ArmConstants.QUADRATIC_B * Math.pow((2 * ArmConstants.QUADRATIC_A), -1) < 0) {}

    final double calculatedArmAngle =
        ArmConstants.QUADRATIC_A * Math.pow(speakerDistance.get(), 2)
            + ArmConstants.QUADRATIC_B * speakerDistance.get()
            + ArmConstants.QUADRATIC_C;

    // and reference them for here so they can update nicely.

    // if (Math.abs(calculatedArmAngle) < 1.753 && Math.abs(calculatedArmAngle) > 0.09) {
    //   System.err.println(
    //       "ERROR: The calculated SpeakerShot angle is outside of the allowed range for the
    // arm.");
    //   return Optional.empty();
    // }

    if (speakerDistance.isEmpty()) {
      System.err.println("WARNING: No speakerDistance detected for SpeakerShot.");
      return Optional.empty();
    }

    return Optional.of(calculatedArmAngle);
  }
}
//  TODO check if .until needed:
//          .until()
//             () -> {
//               return arm.getIsFinished();
//             });`
