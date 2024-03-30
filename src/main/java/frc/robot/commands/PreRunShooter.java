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
// GNU General Public License for more details.s

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.flywheel.Flywheel;

// Moves the note so that it is detected by the conveySensor but not shooterSensor
public class PreRunShooter extends Command {

  private Flywheel flywheel;
  private boolean slowerDefault;

  public PreRunShooter(Flywheel flywheel) {
    this(flywheel, false);
  }

  public PreRunShooter(Flywheel flywheel, boolean slowRun) {
    this.flywheel = flywheel;
    this.slowerDefault = slowRun;
    addRequirements(flywheel);
  }

  // Called once at the beginning
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!slowerDefault) {
      flywheel.runVelocity(-Constants.FlywheelConstants.shootingVelocity);
    } else if (slowerDefault) {
      flywheel.runVelocity(-Constants.FlywheelConstants.slowShootingVelocity);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.runVelocity(0);
    flywheel.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
