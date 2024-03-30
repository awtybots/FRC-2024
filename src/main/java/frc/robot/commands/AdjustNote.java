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
import frc.robot.subsystems.intake.Intake;

// Moves the note so that it is detected by the conveySensor but not shooterSensor
public class AdjustNote extends Command {

  private Intake intake;
  private int phase =
      1; // 1 is moving note to shootersensor, 2 is moving note away from shooterSensor, 3 = done
  private double forwardsIntakeSpeed = 1;
  private double backwardsIntakeSpeed = 1;

  private long phase2StartTime;

  public AdjustNote(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called once at the beginning
  @Override
  public void initialize() {
    phase = 1;
    phase2StartTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (phase == 1) { // go to shooter sensor untill detected
      intake.runPercentSpeed(forwardsIntakeSpeed);
      if (intake.getShooterProximity()) {
        phase = 2;
        phase2StartTime = System.currentTimeMillis();
      }

    } else if (phase == 2) { // go backwards untill not detected
      long phase2RunningTime = System.currentTimeMillis() - phase2StartTime;
      if (phase2RunningTime > 350
          && intake.getIsStalled()) { // get the note unstuck if it is stuck in the shooter
        intake.runPercentSpeed(-0.12);
      } else {
        intake.runPercentSpeed(-backwardsIntakeSpeed);
      }
      if (!intake.getShooterProximity()) {
        phase = 3;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runPercentSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return phase == 3;
  }
}
