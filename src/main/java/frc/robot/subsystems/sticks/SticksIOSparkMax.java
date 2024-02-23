// Copyright 2021-2024 FRC 6328, FRC 5829
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

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SticksConstants;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class SticksIOSparkMax implements SticksIO {
  private static final double GEAR_RATIO = 50; // 50:1 //TODO ratio needed

  private final CANSparkMax motor = // ! Unknown if there will be two or not
      new CANSparkMax(SticksConstants.kSticksMotorId, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkPIDController pid = motor.getPIDController();

  private double targetAngle = SticksConstants.initialAngle; // Radians, just a default value.

  public SticksIOSparkMax() {
    motor.restoreFactoryDefaults();

    motor.setCANTimeout(250);

    // encoder.setPosition(SticksConstants.initialAngle);

    // motor.enableVoltageCompensation(12.0);

    motor.setSmartCurrentLimit(SticksConstants.kCurrentLimit);

    // motor.burnFlash();
  }

  @Override
  public void updateInputs(SticksIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.targetPositionRad = targetAngle;
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  // @Override
  // public void setVelocity(double velocityRadPerSec, double ffVolts) {
  //   pid.setReference(
  //       Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
  //       ControlType.kVelocity,
  //       0,
  //       ffVolts,
  //       ArbFFUnits.kVoltage);
  // }

  @Override
  public void setTargetAngle(double angle) {
    // targetAngle = MathUtil.clamp(angle, SticksConstants.minAngle, SticksConstants.maxAngle);
    targetAngle = angle;
    pid.setReference(
        Units.radiansToRotations(targetAngle) * GEAR_RATIO,
        ControlType.kPosition,
        0,
        0,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}
