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

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;

public class ReverseIntakeFlywheel extends Command {

  private Intake intake;
  private Flywheel flywheel;

  public ReverseIntakeFlywheel(Intake intake, Flywheel flywheel) {
    this.intake = intake;
    this.flywheel = flywheel;
    addRequirements(intake, flywheel);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double targetRPM = -Constants.FlywheelConstants.slowShootingVelocity;

    flywheel.runVelocity(targetRPM);
    intake.runPercentSpeed(1);
  }

  @Override
  public void end(boolean interrupted) {
    flywheel.runVelocity(0);
    intake.runPercentSpeed(0);
  }
}
