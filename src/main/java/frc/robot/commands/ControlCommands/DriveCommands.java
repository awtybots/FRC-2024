// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
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
import frc.robot.Constants.EnvironmentalConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;
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
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          double linearMagnitude;
          Rotation2d linearDirection;
          double omega;
          boolean SlowMode = drive.isSlowMode(); // bandaid solution

          // Apply deadband
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
                    / 2;
            linearDirection =
                new Rotation2d(
                    MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND),
                    MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND));
            omega = (MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND)) / 2;
          }

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          boolean isFlipped = // TODO use this properly
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

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
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  drive.getRotation().plus(new Rotation2d(Math.PI))));
          //   isFlipped
          //       ? drive.getRotation().plus(new Rotation2d(Math.PI))
          //       : drive.getRotation()));
        },
        drive);
  }

  public static Command runOverClosestNote(Drive sDrive, Intake sIntake, Flywheel sFlywheel) {
    return Commands.run(
        () -> {
          double[][] notePositions = EnvironmentalConstants.notePositions;
          Translation2d closestNotePosition = new Translation2d();

          Translation2d currentPostition =
              new Translation2d(sDrive.getPose().getX(), sDrive.getPose().getY());
          double distance = 0;

          for (int i = 0; i < notePositions.length; i++) {
            double x = notePositions[i][0];
            double y = notePositions[i][1];
            Translation2d checkingNotePostition = new Translation2d(x, y);
            double newDistance = currentPostition.getDistance(checkingNotePostition);

            if (newDistance < distance || distance == 0) {
              distance = currentPostition.getDistance(checkingNotePostition);
              closestNotePosition =
                  new Translation2d(
                      currentPostition.getX() + (x - currentPostition.getX()) / 2,
                      currentPostition.getY() + (y - currentPostition.getY()) / 2);
            }
          }

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          sDrive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  closestNotePosition.getX() * sDrive.getMaxLinearSpeedMetersPerSec(),
                  closestNotePosition.getY() * sDrive.getMaxLinearSpeedMetersPerSec(),
                  0,
                  isFlipped
                      ? sDrive.getRotation().plus(new Rotation2d(Math.PI))
                      : sDrive.getRotation()));

          IntakeShooterControls.intakeShooterDrive(sIntake, sFlywheel, () -> 0, () -> 1, () -> true)
              .withTimeout(3);

          sDrive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  currentPostition.getX() * sDrive.getMaxLinearSpeedMetersPerSec(),
                  currentPostition.getY() * sDrive.getMaxLinearSpeedMetersPerSec(),
                  0,
                  isFlipped
                      ? sDrive.getRotation().plus(new Rotation2d(Math.PI))
                      : sDrive.getRotation()));
        },
        sDrive,
        sIntake,
        sFlywheel);
  }
}
