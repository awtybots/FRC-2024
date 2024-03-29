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

package frc.robot.subsystems.sticks;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnvironmentalConstants;
import frc.robot.Constants.SticksConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Sticks extends SubsystemBase {
  private final SticksIO io;
  private final SticksIOInputsAutoLogged inputs = new SticksIOInputsAutoLogged();

  // private final SimpleMotorFeedforward ffModel;

  /** Creates a new Sticks. */
  public Sticks(SticksIO io) {
    this.io = io;

    switch (EnvironmentalConstants.currentMode) {
      case REAL:
        io.configurePID(SticksConstants.kP, SticksConstants.kI, SticksConstants.kD);
      case REPLAY:
        // ffModel = new SimpleMotorFeedforward(SticksConstants.ks, SticksConstants.kv);
        io.configurePID(SticksConstants.kP, SticksConstants.kI, SticksConstants.kD);
        break;
      case SIM:
        // ffModel = new SimpleMotorFeedforward(SticksConstants.ks, SticksConstants.kv);
        io.configurePID(SticksConstants.kP, SticksConstants.kI, SticksConstants.kD);
        break;
      default:
        // ffModel = new SimpleMotorFeedforward(SticksConstants.ks, SticksConstants.kv);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Sticks", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. Not used. */
  // public void runVelocity(double velocityRPM) {
  //   var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
  //   io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

  //   // Log Sticks setpoint
  //   Logger.recordOutput("Sticks/SetpointRPM", velocityRPM);
  // }

  /**
   * Sets the targeted PID angle.
   *
   * @param position Angle in radians.
   */
  public void runTargetAngle(double position) {
    io.setTargetAngle(position);
  }

  /** Sets a target velocity. */
  // public void runTargetVelocity(double targetVelocity) {
  //   io.setTargetAngle(
  //       inputs.targetPositionRad
  //           + 0.02 // correct cycle time here needed
  //               * targetVelocity);
  // }

  /** Stops the Sticks. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }
}
