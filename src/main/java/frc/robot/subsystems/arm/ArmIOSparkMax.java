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

  private double targetAngle = ArmConstants.initialAngle;

  // REMEMBER

  public ArmIOSparkMax() {
    // leftMotor.restoreFactoryDefaults();
    // rightMotor.restoreFactoryDefaults();

    mathPid =
        new PIDController(
            Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD);

    leftMotor.setCANTimeout(250);
    rightMotor.setCANTimeout(250);

    // set wrapping on leftMotorAbsoluteEncoder so it does not go from 0 to 2 pi, causing a jump

    // leftRelativeEncoder.setPosition(0.0);

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);

    // set both motors to brake
    leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
    rightMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);

    rightMotor.follow(leftMotor, true);
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
    inputs.positionRad = leftAbsoluteEncoder.getPosition() * Math.PI * 2.0;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(leftAbsoluteEncoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.currentAmps = new double[] {leftMotor.getOutputCurrent()};
    inputs.targetPositionRad = targetAngle;
  }

  // @Override
  // public void setVoltage(double volts) {
  //   leftMotor.setVoltage(volts);
  //   rightMotor.setVoltage(volts);
  // }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    // pid.setReference(
    //     Units.rotationsToRadians(velocityRadPerSec) * GEAR_RATIO,
    //     ControlType.kVelocity,
    //     0,
    //     ffVolts,
    //     ArbFFUnits.kVoltage);
  }

  @Override
  /** Set the target angle. In radians. */
  public void setTargetAngle(double angle) {
    targetAngle = MathUtil.clamp(angle, ArmConstants.minimumAngle, ArmConstants.maximumAngle);
    //   pid.setReference(
    //       targetAngle * Math.PI * 2.0,
    //       ControlType.kPosition,
    //       0,
    //       ArmConstants.kWeightBasedFF
    //           * Math.sin(Units.rotationsToRadians(leftAbsoluteEncoder.getPosition() /
    // GEAR_RATIO))
    //           * 0,
    //       ArbFFUnits.kVoltage);

    leftMotor.set(
        MathUtil.clamp(
            mathPid.calculate(leftAbsoluteEncoder.getPosition() * Math.PI * 2.0, targetAngle),
            -ArmConstants.kMaxOutput,
            ArmConstants.kMaxOutput));
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
