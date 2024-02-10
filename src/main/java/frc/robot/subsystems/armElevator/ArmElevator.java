// Copyright 2021-2024 FRC 6328, FRC 5289
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

package frc.robot.subsystems.armElevator;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmElevatorConstants;
import frc.robot.Constants.EnvironmentalConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmElevator extends SubsystemBase {
  private final ArmElevatorIO io;
  private final ArmElevatorIOInputsAutoLogged inputs = new ArmElevatorIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  /** Creates a new ArmElevator. */
  public ArmElevator(ArmElevatorIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (EnvironmentalConstants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(ArmElevatorConstants.ks, ArmElevatorConstants.kv);
        io.configurePID(ArmElevatorConstants.kP, ArmElevatorConstants.kI, ArmElevatorConstants.kD);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(ArmElevatorConstants.ks, ArmElevatorConstants.kv);
        io.configurePID(ArmElevatorConstants.kP, ArmElevatorConstants.kI, ArmElevatorConstants.kD);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(ArmElevatorConstants.ks, ArmElevatorConstants.kv);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ArmElevator", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log ArmElevator setpoint
    Logger.recordOutput("ArmElevator/SetpointRPM", velocityRPM);
  }

  /** Stops the ArmElevator. */
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
