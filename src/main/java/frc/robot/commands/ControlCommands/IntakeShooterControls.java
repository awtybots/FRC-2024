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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public class IntakeShooterControls {
  private static final double DEADBAND = 0.3;

  public static Command intakeShooterDefaultCommand(
      Intake intake, DoubleSupplier rightTriggerSupplier) {

    return Commands.run(
        () -> {
          if (Math.abs(rightTriggerSupplier.getAsDouble()) > DEADBAND) {
            intake.runPercentSpeed(
                Constants.IntakeConstants.percentPower * rightTriggerSupplier.getAsDouble());
          }
        },
        intake);
  }
}
