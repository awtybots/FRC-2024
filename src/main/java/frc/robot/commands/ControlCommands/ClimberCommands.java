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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.Climber;

public class ClimberCommands {
  // private static final double DEADBAND = 0.3;

  private static final double MAX_MS = 0.01;

  private ClimberCommands() {}

  /** Wrist command using one axis of a joystick (controlling wrist velocity). */
  public static Command buttonDrive(Climber climber, Trigger leftTrigger, Trigger rightTrigger) {
    return Commands.run(
        () -> {
          boolean left = leftTrigger.getAsBoolean();
          boolean right = rightTrigger.getAsBoolean();
          double magnitude = 0;

          if (left && right) { // there's probably a better way of doing this
            magnitude = 0;
          } else if ((left) && (!right)) {
            magnitude = 1;
          } else if ((!left) && (right)) {
            magnitude = -1;
          } else if ((!left) && (!right)) {
            magnitude = 0;
          }

          // Calcaulate new rotational velocity
          double position = magnitude * MAX_MS;

          // Send command to wrist subsystem to run wrist
          climber.runTargetPosition(position);
        },
        climber);
  }
}
