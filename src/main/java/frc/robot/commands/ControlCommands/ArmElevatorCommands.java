// Copyright 2021-2024 FRC 6328, FRC 5829
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.armElevator.ArmElevator;
import java.util.function.DoubleSupplier;

public class ArmElevatorCommands {
  private static final double DEADBAND = 0.3;
  private static final double MAXINCHESPERSECOND = 3;

  private ArmElevatorCommands() {}

  /** Wrist command using one axis of a joystick (controlling wrist velocity). */
  public static Command triggerDrive(
      ArmElevator armElevator,
      DoubleSupplier leftTriggerSupplier,
      DoubleSupplier rightTriggerSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband (i.e. min DEADBAND max 1.0)
          double stickMagnitude =
              MathUtil.applyDeadband(leftTriggerSupplier.getAsDouble(), DEADBAND)
                  + -MathUtil.applyDeadband(rightTriggerSupplier.getAsDouble(), DEADBAND);

          // Calcaulate new rotational velocity
          double rotationalVelocity = stickMagnitude * MAXINCHESPERSECOND;

          // Send command to wrist subsystem to run wrist
          armElevator.runTargetVelocity(rotationalVelocity);
        },
        armElevator);
  }
}
