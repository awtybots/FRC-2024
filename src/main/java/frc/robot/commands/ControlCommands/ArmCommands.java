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

package frc.robot.commands.ControlCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import java.util.function.DoubleSupplier;

public class ArmCommands {
  private static final double DEADBAND = 0.3;
  private static final double MAXRPM = 0.1;

  private ArmCommands() {}

  /** Arm command using one axis of a joystick (controlling arm velocity). */
  public static Command joystickDrive(Arm arm, DoubleSupplier ySupplier) {
    return Commands.run(
        () -> {
          // Apply deadband (min DEADBAND, max 1.0)
          double stickMagnitude = MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND);

          // Square values, so that it's easier to control at lower speeds
          // double sign = Math.copySign(1, stickMagnitude);
          // stickMagnitude = stickMagnitude * stickMagnitude * sign;

          // Calcaulate new rotational velocity
          double rotationalVelocity = stickMagnitude * MAXRPM;

          // Send command to arm to run arm
          arm.runVelocity(rotationalVelocity);
        },
        arm);
  }
}
