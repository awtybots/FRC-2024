// Copyright 2016-2024 FRC 5829, FRC 6328
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

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.EnvironmentalConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// ! TODO Seperate target positions but they're controlled by one so that they can be controlled
// ! seperately from smartdashboard

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (EnvironmentalConstants.currentMode) {
      case REAL:
        io.configurePID(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);
      case REPLAY:
        io.configurePID(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);
        break;
      case SIM:
        io.configurePID(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);
        break;
      default:
        break;
    }
    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Climber/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Sets the targeted PID position.
   *
   * @param position meters.
   */
  public void runTargetPosition(double position) {
    io.setTargetPosition(position);
  }

  /**
   * Sets the targeted PID position.
   *
   * @param position meters.
   */
  public void runTargetPosition(double positionLeft, double positionRight) {
    io.setTargetPosition(positionLeft, positionRight);
  }

  /** Stops the Climber. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in ms-1. */
  @AutoLogOutput(key = "Climber/LeftVelocity")
  public double getLeftVelocity() {
    return inputs.leftVelocity;
  }

  /** Returns the current right velocity in ms-1. */
  @AutoLogOutput(key = "Climber/RightVelocity")
  public double getRightVelocity() {
    return inputs.rightVelocity;
  }

  /** Returns the current left position in m. */
  @AutoLogOutput(key = "Climber/LeftPositionRad")
  public double getLeftPosition() {
    return inputs.leftPosition;
  }

  /** Returns the current right position in m. */
  @AutoLogOutput(key = "Climber/RightPositionRad")
  public double getRightPosition() {
    return inputs.rightPosition;
  }

  /** Returns the current target position in m. */
  // TODO separate target positions properly, with smartdashboard reset and movement buttons
  @AutoLogOutput(key = "Climber/TargetPosition")
  public double getTargetPosition() {
    return inputs.targetPosition;
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
