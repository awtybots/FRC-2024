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
import frc.robot.subsystems.intake.Intake;

// Moves the note so that it is detected by the conveySensor but not shooterSensor

public class IntakeNote extends Command {

  private Intake intake;

  public IntakeNote(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called once at the beginning
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (intake.getConveyerProximity()) {
      intake.runPercentSpeed(0);
    } else {
      intake.runPercentSpeed(Constants.IntakeConstants.percentPower);
    }
  }

  // Run AdjustNote with a timeout of 4 seconds
  @Override
  public void end(boolean interrupted) {
    // AdjustNote adjustNoteCommand = new AdjustNote(intake);
    // adjustNoteCommand.withTimeout(1.5).schedule();
  }

  @Override
  public boolean isFinished() {
    return intake.getConveyerProximity();
  }
}
