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
import frc.robot.subsystems.arm.Arm;

public class ArmCommands {
  // private static final double DEADBAND = 0.3;
  private static final double MAXRPM = 0.01; // definitely wrong

  private ArmCommands() {}

  /** Arm command using one axis of a joystick (controlling arm velocity). */
  public static Command joystickDrive(Arm arm, Trigger povUPTrigger, Trigger povDownTrigger) {
    return Commands.run(
        () -> {
          boolean up = povUPTrigger.getAsBoolean();
          boolean down = povDownTrigger.getAsBoolean();
          double magnitude = 0;

          if (up && down) { // there's probably a better way of doing this
            magnitude = 0;
          } else if ((up) && (!down)) {
            magnitude = 1;
          } else if ((!up) && (down)) {
            magnitude = -1;
          } else if ((!up) && (!down)) {
            magnitude = 0;
          }

          // Calcaulate new rotational velocity
          double addTargetAngle = magnitude * MAXRPM;

          // Send command to arm to run arm
          arm.addTargetAngle(addTargetAngle);
        },
        arm);
  }
}
