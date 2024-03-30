// Copyright 2016-2024 FRC 6328, FRC 5829
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

package frc.robot.commands.ControlCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.17;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Boolean shouldPointAtSpeaker) {
    return Commands.run(
        () -> {
          double linearMagnitude;
          Rotation2d linearDirection;
          double omega;
          boolean SlowMode = drive.isSlowMode();

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          // Apply deadband and slowMode
          if (!SlowMode) {
            linearMagnitude =
                MathUtil.applyDeadband(
                    Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
            linearDirection =
                new Rotation2d(
                    MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND),
                    MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND));
            omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
          } else {
            linearMagnitude =
                (MathUtil.applyDeadband(
                        Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND))
                    / 2.0;
            linearDirection =
                new Rotation2d(
                    MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND) / 2.0,
                    MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND) / 2.0);
            omega = (MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND) / 2.0);
          }

          // TODO Ranging control?
          // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-aiming-and-ranging
          if (shouldPointAtSpeaker) {
            // TEST A really bad way of implementing this.
            // double[] currentPosition = {drive.getPose().getX(), drive.getPose().getY()};
            // double[] speakerPosition = {0.25, 5.55};
            // double targetAngleRadians =
            //     Math.atan(
            //         (currentPosition[1] - speakerPosition[1])
            //             / (currentPosition[0] - speakerPosition[0]));
            // omega = targetAngleRadians;

            // TEST Implemented using a limelight, better. Double check pls.
            double kP = 0.035;
            // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost
            // edge of
            // your limelight 3 feed, tx should return roughly 31 degrees. Converted to radians.
            double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * (Math.PI / 180);

            targetingAngularVelocity *= kP;
            // invert since tx is positive when the target is to the right of the crosshair
            targetingAngularVelocity *= -1.0;

            targetingAngularVelocity =
                MathUtil.clamp(
                    targetingAngularVelocity,
                    -drive.getMaxAngularSpeedRadPerSec(),
                    drive.getMaxAngularSpeedRadPerSec());
            omega = targetingAngularVelocity;
          }

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  1.25 * omega * drive.getMaxAngularSpeedRadPerSec() / 2.3,
                  isFlipped // TEST TODO
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }
}
