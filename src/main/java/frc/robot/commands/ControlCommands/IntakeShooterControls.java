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
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeShooterControls {
  private static final double DEADBAND = 0.3;
  private static final double MAXINCHESPERSECOND = 3;

  private IntakeShooterControls() {}

  public static Command intakeShooterDrive(
      Intake intake,
      Flywheel flywheel,
      Arm arm,
      DoubleSupplier leftTriggerSupplier,
      DoubleSupplier rightTriggerSupplier,
      BooleanSupplier leftBumperSupplier,
      BooleanSupplier rightBumperSupplier) {
    // Left trigger runs outtake, right trigger triggers intake, and right bumper triggers the
    // shooter (and the intake once at velocity).
    // Intake stops on detection of note in first mode.
    return Commands.run(
        () -> {
          /*
          if (leftBumperSupplier.getAsBoolean()) {
            // flywheel.runVelocity(-Constants.FlywheelConstants.shootingVelocity);
          }
          if (rightBumperSupplier.getAsBoolean()) {

          } else {

            // double fwdSpeed = leftTriggerSupplier.getAsDouble();
            double revSpeed = rightTriggerSupplier.getAsDouble();
            // fwdSpeed = MathUtil.applyDeadband(fwdSpeed, DEADBAND);
            // revSpeed = MathUtil.applyDeadband(revSpeed, DEADBAND);

            double stickMagnitude = -revSpeed;
            stickMagnitude =
                stickMagnitude * stickMagnitude * stickMagnitude; // more control over lower speeds

            if (Math.abs(revSpeed) > DEADBAND) {
              intake.runPercentSpeed(Constants.IntakeConstants.percentPower * stickMagnitude);
            }
          }
          */
          if (Math.abs(rightTriggerSupplier.getAsDouble()) > DEADBAND) {
            intake.runPercentSpeed(
                -Constants.IntakeConstants.percentPower * rightTriggerSupplier.getAsDouble());
          }
        },
        intake,
        flywheel);
  }
}
