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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import java.util.function.DoubleSupplier;

public class ArmCommands {
  private static final double DEADBAND = 0.3;
  private static final double MAXRPM = 5; // TODO idk what a realistic one is so this is roughly 90 degrees per 2 seconds

  private ArmCommands() {}

  /**
   * Arm command using one axis of a joystick (controlling arm velocity).
   */
  public static Command joystickDrive(
      Arm arm,
      DoubleSupplier ySupplier) {
    return Commands.run(
        () -> {
          // Apply deadband (min DEADBAND, max 1.0)
          double stickMagnitude =
              MathUtil.applyDeadband(
                  ySupplier.getAsDouble(), DEADBAND);

          // Square values, so that it's easier to control at lower speeds
          stickMagnitude = stickMagnitude * stickMagnitude;

          // Calcaulate new rotational velocity
          double rotationalVelocity = stickMagnitude * MAXRPM;

          // Send command to arm to run arm
          arm.runTargetVelocity(rotationalVelocity);
        },
        arm);
  }
}
