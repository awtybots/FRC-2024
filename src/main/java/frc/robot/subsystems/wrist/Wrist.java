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

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnvironmentalConstants;
import frc.robot.Constants.WristConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  /** Creates a new Wrist. */
  public Wrist(WristIO io) {
    this.io = io;

    switch (EnvironmentalConstants.currentMode) {
      case REAL:
        io.configurePID(WristConstants.kP, WristConstants.kI, WristConstants.kD);
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(WristConstants.ks, WristConstants.kv);
        io.configurePID(WristConstants.kP, WristConstants.kI, WristConstants.kD);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(WristConstants.ks, WristConstants.kv);
        io.configurePID(WristConstants.kP, WristConstants.kI, WristConstants.kD);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(WristConstants.ks, WristConstants.kv);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log Wrist setpoint
    Logger.recordOutput("Wrist/SetpointRPM", velocityRPM);
  }

  /**
   * Sets the targeted PID angle.
   *
   * @param position Angle in radians.
   */
  public void runTargetAngle(double position) {
    io.setTargetAngle(position);
  }

  public void runTargetVelocity(double targetVelocity) {
    io.setTargetAngle(
        inputs.targetPositionRad
            + 0.02 // TODO correct cycle time here needed
                * targetVelocity);
  }

  /** Stops the Wrist. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput(key = "Wrist/Velocity(rotationsperminute)")
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Get current recorded angle */
  @AutoLogOutput(key = "Wrist/Angle")
  public double getAngle() {
    return inputs.positionRad;
  }

  @AutoLogOutput(key = "Wrist/TargetAngle")
  public double getTargetAngle() {
    return inputs.targetPositionRad;
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }
}
