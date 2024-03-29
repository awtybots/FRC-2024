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

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ClimberConstants;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ClimberIOSparkMax implements ClimberIO {
  private static final double GEAR_RATIO = 50.0; // 49:1 in reality but too late

  private final CANSparkMax leftMotor =
      new CANSparkMax(ClimberConstants.kLeftClimberMotorId, MotorType.kBrushless);
  private final CANSparkMax rightMotor =
      new CANSparkMax(ClimberConstants.kRightClimberMotorId, MotorType.kBrushless);

  private final RelativeEncoder leftRelativeEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightRelativeEncoder = rightMotor.getEncoder();

  private final SparkPIDController leftPID = leftMotor.getPIDController();
  private final SparkPIDController rightPID = rightMotor.getPIDController();

  private double targetPosition = ClimberConstants.initialPosition;

  private double targetPositionLeft = ClimberConstants.initialPosition;
  private double targetPositionRight = ClimberConstants.initialPosition;

  public ClimberIOSparkMax() {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setCANTimeout(250);
    rightMotor.setCANTimeout(250);

    leftMotor.setInverted(true);

    leftMotor.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);
    rightMotor.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);

    // leftMotor.burnFlash();
    // rightMotor.burnFlash();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftPosition =
        ClimberConstants.gearCircumfrence * (leftRelativeEncoder.getPosition() / GEAR_RATIO);
    inputs.leftVelocity =
        ClimberConstants.gearCircumfrence * (leftRelativeEncoder.getVelocity() / GEAR_RATIO);
    inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.leftCurrentAmps = new double[] {leftMotor.getOutputCurrent()};

    inputs.rightPosition =
        ClimberConstants.gearCircumfrence * (rightRelativeEncoder.getPosition() / GEAR_RATIO);
    inputs.rightVelocity =
        ClimberConstants.gearCircumfrence * (rightRelativeEncoder.getVelocity() / GEAR_RATIO);
    inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    inputs.rightCurrentAmps = new double[] {rightMotor.getOutputCurrent()};

    inputs.targetPosition = targetPosition;
  }

  @Override
  public void setVoltage(double volts) {
    leftMotor.setVoltage(volts);
    rightMotor.setVoltage(volts);
  }

  @Override
  public void setTargetPosition(double position) {
    targetPosition =
        MathUtil.clamp(targetPosition, ClimberConstants.minPosition, ClimberConstants.maxPosition);

    leftPID.setReference(
        (position / ClimberConstants.gearCircumfrence) * GEAR_RATIO,
        ControlType.kPosition,
        0,
        0,
        ArbFFUnits.kVoltage);

    rightPID.setReference(
        (position / ClimberConstants.gearCircumfrence) * GEAR_RATIO,
        ControlType.kPosition,
        0,
        0,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTargetPosition(double positionLeft, double positionRight) {
    targetPositionLeft =
        MathUtil.clamp(positionLeft, ClimberConstants.minPosition, ClimberConstants.maxPosition);
    ;
    targetPositionRight =
        MathUtil.clamp(positionRight, ClimberConstants.minPosition, ClimberConstants.maxPosition);
    ;

    leftPID.setReference(
        (positionLeft / ClimberConstants.gearCircumfrence) * GEAR_RATIO,
        ControlType.kPosition,
        0,
        0,
        ArbFFUnits.kVoltage);

    rightPID.setReference(
        (positionRight / ClimberConstants.gearCircumfrence) * GEAR_RATIO,
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
    leftPID.setP(kP, 0);
    leftPID.setI(kI, 0);
    leftPID.setD(kD, 0);
    leftPID.setFF(0, 0);

    rightPID.setP(kP, 0);
    rightPID.setI(kI, 0);
    rightPID.setD(kD, 0);
    rightPID.setFF(0, 0);
  }
}
