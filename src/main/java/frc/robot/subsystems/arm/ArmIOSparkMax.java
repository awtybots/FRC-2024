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

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ArmIOSparkMax implements ArmIO {
  private static final double GEAR_RATIO = 40.0 * (60.0 / 12.0); // May be reciprocal

  private final CANSparkMax leftMotor =
      new CANSparkMax(ArmConstants.kLeftArmMotorId, MotorType.kBrushless);
  private final CANSparkMax rightMotor =
      new CANSparkMax(ArmConstants.kRightArmMotorId, MotorType.kBrushless);

  private final RelativeEncoder leftRelativeEncoder = leftMotor.getEncoder();

  private final SparkAbsoluteEncoder leftAbsoluteEncoder =
      leftMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

  private final SparkPIDController pid = leftMotor.getPIDController();

  private double targetAngle = 0; // Radians, just a default value.

  public ArmIOSparkMax() {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setCANTimeout(250);
    rightMotor.setCANTimeout(250);

    // leftRelativeEncoder.setPosition(0.0);

    // left.setInverted(false);
    // right.setInverted(false);

    leftMotor.enableVoltageCompensation(12.0);
    rightMotor.enableVoltageCompensation(12.0);

    leftMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
    rightMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);

    rightMotor.follow(leftMotor, true);

    // leftMotor.burnFlash();
    // rightMotor.burnFlash();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(leftRelativeEncoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(leftRelativeEncoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.currentAmps = new double[] {leftMotor.getOutputCurrent()};
    inputs.targetPositionRad = targetAngle;
  }

  @Override
  public void setVoltage(double volts) {
    leftMotor.setVoltage(volts);
    rightMotor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTargetAngle(double angle) {
    targetAngle = MathUtil.clamp(angle, ArmConstants.minimumAngle, ArmConstants.maximumAngle);
    pid.setReference(
        Units.radiansToRotations(targetAngle) * GEAR_RATIO,
        ControlType.kPosition,
        0,
        0,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }

  // TODO: The following commented code is from the ArmSubsystem - see if it needs to be integrated
  // here
  /*
  public void setRotation(double value) {
    armHeight = value;
  }

  public void setDegrees(double degrees) {
    double encoderunits =
        Convert.angleToEncoderPos(degrees, kArmGearRatio, Encoder.RevRelativeEncoder);
    armHeight = encoderunits;
  }

  public double getAngle() {
    final double rawRevs = mRightArmEncoder.getPosition();
    final double theta =
        Convert.encoderPosToAngle(rawRevs, kArmGearRatio, Encoder.RevRelativeEncoder);
    return ArmConstants.startingAngle - theta;
  }

  public void resetEncoderValue() {
    armHeight = 0;
    mRightArmEncoder.setPosition(armHeight);
  }

  public void drive(double pct) {

    // MathUtil.clamp(armHeight, ArmConstants.minimumHeight, getMaximumRotation());
    armHeight += pct * ArmConstants.armConversion;
  }

  public boolean isFinished() {
    return Math.abs(mRightArmEncoder.getPosition() - armHeight) < Presets.ArmThreshold;
  }

  @Override
  public void periodic() {
    mRightArmPIDController.setReference(armHeight, CANSparkMax.ControlType.kPosition, 0);
    // SmartDashboard.putNumber(
    //         "Arm Error",
    //         Convert.encoderPosToAngle(
    //                 mRightArmEncoder.getPosition() - armHeight, kArmGearRatio,
    // Encoder.RevRelativeEncoder));
    // SmartDashboard.putNumber("Arm angle", -(this.getAngle() - Arm.startingAngle));
  }
   */

}
