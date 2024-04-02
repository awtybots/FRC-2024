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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;

public class ShootClose { // bumper pressed against the wall

  public static Command run(Arm arm) {
    return Commands.run(
            () -> {
              // Position preset settings
              double ARMANGLE = 0.747;

              arm.runTargetAngle(ARMANGLE);
            },
            arm)
        .until(
            () -> {
              return arm.getIsFinished();
            });
  }
}
