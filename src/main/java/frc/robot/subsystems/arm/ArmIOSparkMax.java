// Copyright 2016-2024 FRC 6328, FRC 5829//
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

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ArmIOSparkMax implements ArmIO {
  private static final double GEAR_RATIO = 1; // May be reciprocal

  private final CANSparkMax leftMotor =
      new CANSparkMax(ArmConstants.kLeftArmMotorId, MotorType.kBrushless);
  private final CANSparkMax rightMotor =
      new CANSparkMax(ArmConstants.kRightArmMotorId, MotorType.kBrushless);

  // private final RelativeEncoder leftRelativeEncoder = leftMotor.getEncoder();

  private final SparkAbsoluteEncoder leftAbsoluteEncoder =
      leftMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

  // private final SparkPIDController pid = leftMotor.getPIDController();

  private final PIDController mathPid;

  private double targetAngle =
      Constants.ArmConstants.uprightAngle; // 2.2// Radians, just a default value

  private double lastEncoderReading = 0.4;

  private double calculatedPID;

  // REMEMBER THE ALAMO

  public ArmIOSparkMax() {
    // leftMotor.restoreFactoryDefaults();
    // rightMotor.restoreFactoryDefaults();

    mathPid =
        new PIDController(
            Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD);

    leftMotor.setCANTimeout(250);
    rightMotor.setCANTimeout(250);

    leftAbsoluteEncoder.setInverted(true);

    // set wrapping on leftMotorAbsoluteEncoder so it does not go from 0 to 2 pi, causing a jump

    // leftRelativeEncoder.setPosition(0.0);

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);

    // set both motors to coast
    leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
    rightMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);

    rightMotor.follow(leftMotor, true);

    lastEncoderReading = leftAbsoluteEncoder.getPosition();
    // pid.setFeedbackDevice(leftAbsoluteEncoder);
    // pid.setPositionPIDWrappingEnabled(true);
    // pid.setPositionPIDWrappingMaxInput(1);
    // pid.setPositionPIDWrappingMinInput(0);

    // set ranges for pid and absolutencoders to - pi and pi

    // leftMotor.burnFlash();
    // rightMotor.burnFlash();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionRad = getSmoothedPosition() * Math.PI * 2.0;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(leftAbsoluteEncoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.currentAmps = new double[] {leftMotor.getOutputCurrent()};
    inputs.targetPositionRad = targetAngle;
  }

  private double getSmoothedPosition() {
    if (Math.abs(leftAbsoluteEncoder.getPosition()) < 0.02 && Math.abs(lastEncoderReading) > 0.1) {
      return lastEncoderReading;
    }
    return leftAbsoluteEncoder.getPosition();
  }

  public double getAngleFromVertical() {
    return (getSmoothedPosition() * Math.PI * 2.0 - Constants.ArmConstants.uprightAngle);
  }

  @Override
  public void setVoltage(double volts) {
    leftMotor.setVoltage(volts);
    rightMotor.setVoltage(volts);
  }

  // Returns true if the arm is stationary.
  @Override
  public boolean getIsFinished() {
    return calculatedPID < 0.037;
  }

  @Override
  /** Sets the velocity at the specified velocity. Needs to be run periodically. */
  public void setVelocity(double velocityRPM) {
    setTargetAngle(
        targetAngle
            + 0.02 // TODO correct cycle time here needed
                // * ArmConstants.armConversion
                * Units.rotationsToRadians(velocityRPM));
  }

  @Override
  /** Set the target angle. In radians. */
  public void setTargetAngle(double angle) {
    targetAngle = MathUtil.clamp(angle, ArmConstants.minimumAngle, ArmConstants.maximumAngle);
    // Note that updateMotorSpeeds(); is run periodically, which actually applies these changes.

    //   pid.setReference(
    //       targetAngle * Math.PI * 2.0,
    //       ControlType.kPosition,
    //       0,
    //       ArmConstants.kWeightBasedFF
    //           * Math.sin(Units.rotationsToRadians(leftAbsoluteEncoder.getPosition() /
    // GEAR_RATIO))
    //           * 0,
    //       ArbFFUnits.kVoltage);
  }

  @Override
  /** Update speeds according to PID values from target angle. */
  public void updateMotorSpeeds() {
    calculatedPID = mathPid.calculate(getSmoothedPosition() * Math.PI * 2.0, targetAngle);

    leftMotor.set(
        MathUtil.clamp(
            calculatedPID
                + -Constants.ArmConstants.kWeightBasedFF * Math.sin(getAngleFromVertical()),
            -ArmConstants.kMaxOutput,
            ArmConstants.kMaxOutput));

    lastEncoderReading = leftAbsoluteEncoder.getPosition();
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    mathPid.setP(Constants.ArmConstants.kP);
    mathPid.setI(Constants.ArmConstants.kI);
    mathPid.setD(Constants.ArmConstants.kD);
  }
}
