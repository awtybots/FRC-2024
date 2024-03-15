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

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.EnvironmentalConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    io.configurePID(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

    switch (EnvironmentalConstants.currentMode) {
      case REAL:
        // io.configurePID(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(ArmConstants.ks, ArmConstants.kv);
        // io.configurePID(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(ArmConstants.ks, ArmConstants.kv);
        // io.configurePID(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(ArmConstants.ks, ArmConstants.kv);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public boolean hasReachedDestination() {
    return io.hasReachedDestination();
  }

  /**
   * Run closed loop at the specified velocity.
   *
   * @param velocityRPM The velocity in rotations per minute.
   */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log arm setpoint
  }
  /**
   * @param rightPosition
   */
  public void runTargetVelocity(double targetVelocity) {
    io.setTargetAngle(
        inputs.targetPositionRad
            + 0.02 // TODO correct cycle time here needed
                // * ArmConstants.armConversion
                * Units.rotationsToRadians(targetVelocity));
  }

  /**
   * Sets the targeted PID angle.
   *
   * @param position Angle in radians.
   */
  public void runTargetAngle(double position) {
    io.setTargetAngle(position);
  }

  /** Stops the arm. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput(key = "Arm/VelocityRPM")
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  @AutoLogOutput(key = "Arm/PositionRad")
  public double getPosition() {
    return inputs.positionRad;
  }

  @AutoLogOutput(key = "Arm/TargetPositionRad")
  public double getTargetPosition() {
    return inputs.targetPositionRad;
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }
}
