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

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EnvironmentalConstants;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  public static final Lock odometryLock = new ReentrantLock();

  private final ProximitySensorIO proximitySensorIO;
  private final ProximitySensorIOInputsAutoLogged proximitySensorInputs =
      new ProximitySensorIOInputsAutoLogged();

  /** Creates a new Flywheel. */
  public Intake(IntakeIO io, ProximitySensorIOV3 proximitySensorIO2V3) {
    this.io = io;
    this.proximitySensorIO = proximitySensorIO2V3;

    io.configurePID(
        Constants.IntakeConstants.kP, Constants.IntakeConstants.kI, Constants.IntakeConstants.kD);

    switch (EnvironmentalConstants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    proximitySensorIO.updateInputs(proximitySensorInputs);
    io.updateInputs(inputs);
    odometryLock.unlock();

    Logger.processInputs("Intake", inputs);
    Logger.processInputs("Proximity", proximitySensorInputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the percent speed. */
  public void runPercentSpeed(double percentSpeed) {
    percentSpeed = -percentSpeed;
    io.setPercentSpeed(percentSpeed);

    // Log flywheel setpoint
    Logger.recordOutput("Intake/PercentSpeed", percentSpeed);
  }

  public boolean getConveyerProximity() {
    return proximitySensorInputs.isConveyorSensorTriggered;
  }

  public boolean getShooterProximity() {
    return proximitySensorInputs.isShooterSensorTriggered;
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  public void runFull() {
    io.runFull();
  }

  public boolean getIsStalled() {
    return io.getIsStalled();
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
