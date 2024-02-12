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

package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnvironmentalConstants;
import frc.robot.Constants.FlywheelConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;

    // Switch constants based on mode.
    switch (EnvironmentalConstants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(FlywheelConstants.ks, FlywheelConstants.kv);
        io.configurePID(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(FlywheelConstants.ks, FlywheelConstants.kv);
        io.configurePID(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(FlywheelConstants.ks, FlywheelConstants.kv);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPMTop() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSecTop);
  }

  @AutoLogOutput
  public double getVelocityRPMBottom() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSecBottom);
  }

  /**
   * Returns the current velocity in radians per second. Note: I don't think this will be necessary
   * to include in the Auto Manager at all. Is feedforward even appropriate for this?
   */
  public double getCharacterizationVelocity() {
    return (inputs.velocityRadPerSecTop + inputs.velocityRadPerSecBottom) / 2;
  }
}
