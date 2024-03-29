// Copyright 2021-2024 FRC 5829
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
import frc.robot.Constants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeShooterControls {
  private static final double DEADBAND = 0.3;

  private IntakeShooterControls() {}

  public static Command intakeShooterDrive(
      Intake intake,
      Flywheel flywheel,
      DoubleSupplier leftTriggerSupplier,
      DoubleSupplier rightTriggerSupplier,
      BooleanSupplier bumperSupplier) {
    // Left trigger runs outtake, right trigger triggers intake, and bumper triggers the
    // shooter (and the intake once at velocity).
    // Intake stops on detection of note in first mode.
    return Commands.run(
        () -> {
          if (bumperSupplier.getAsBoolean()) {
            flywheel.runVelocity(-Constants.FlywheelConstants.shootingVelocity);

            double flywheelRPM = flywheel.getVelocityRPMBottom();
            double targetRPM =
                Constants.FlywheelConstants.shootingVelocity
                    + 2000; // TODO find actual max velocity

            if (flywheelRPM > targetRPM) {
              intake.runPercentSpeed(1);
            }

          } else {
            flywheel.stop();

            double fwdSpeed = leftTriggerSupplier.getAsDouble();
            double revSpeed = rightTriggerSupplier.getAsDouble();
            fwdSpeed = MathUtil.applyDeadband(fwdSpeed, DEADBAND);
            revSpeed = MathUtil.applyDeadband(revSpeed, DEADBAND);

            double stickMagnitude = fwdSpeed - revSpeed;
            stickMagnitude =
                stickMagnitude * stickMagnitude * stickMagnitude; // more control over lower speeds

            int proximity = intake.getProximity();
            boolean noteDetected = proximity > 50;

            if (noteDetected) {
              if (stickMagnitude > 0.15) {
                intake.runPercentSpeed(0);
              } else {
                intake.runPercentSpeed(
                    Constants.IntakeConstants.percentPower * stickMagnitude); // allow for extake
              }
            } else {
              intake.runPercentSpeed(Constants.IntakeConstants.percentPower * stickMagnitude);
            }
          }
        },
        intake,
        flywheel);
  }
}
