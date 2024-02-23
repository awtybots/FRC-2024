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

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FlywheelConstants;

public class FlywheelIOSparkMax implements FlywheelIO {
  private static final double GEAR_RATIO = 52.0 / 34.0;

  /*
   Note: I do not believe there will be any situation where the top motor and the bottom motor
   will need to be treated differently (because if they were, they could rip the note apart in
   the current design). The following could also be achieved with a follower motor.
  */
  private final CANSparkMax topShooterMotor =
      new CANSparkMax(FlywheelConstants.kTopFlywheelSparkMaxCanId, MotorType.kBrushless);
  private final RelativeEncoder topShooterEncoder = topShooterMotor.getEncoder();
  private final SparkPIDController topShooterPID = topShooterMotor.getPIDController();

  private final CANSparkMax bottomShooterMotor =
      new CANSparkMax(FlywheelConstants.kBottomFlywheelSparkMaxCanId, MotorType.kBrushless);
  private final RelativeEncoder bottomShooterEncoder = bottomShooterMotor.getEncoder();
  private final SparkPIDController bottomShooterPID = bottomShooterMotor.getPIDController();

  public FlywheelIOSparkMax() {
    topShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.restoreFactoryDefaults();

    topShooterMotor.setCANTimeout(250);
    bottomShooterMotor.setCANTimeout(250);

    topShooterMotor.setInverted(false);
    bottomShooterMotor.setInverted(false); 

    topShooterMotor.enableVoltageCompensation(12.0);
    topShooterMotor.setSmartCurrentLimit(30);
    bottomShooterMotor.enableVoltageCompensation(12.0);
    bottomShooterMotor.setSmartCurrentLimit(30);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.positionRadTop = Units.rotationsToRadians(topShooterEncoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSecTop =
        Units.rotationsPerMinuteToRadiansPerSecond(topShooterEncoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVoltsTop = topShooterMotor.getAppliedOutput() * topShooterMotor.getBusVoltage();
    inputs.currentAmpsTop = new double[] {topShooterMotor.getOutputCurrent()};

    inputs.positionRadBottom =
        Units.rotationsToRadians(bottomShooterEncoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSecBottom =
        Units.rotationsPerMinuteToRadiansPerSecond(bottomShooterEncoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVoltsBottom =
        bottomShooterMotor.getAppliedOutput() * bottomShooterMotor.getBusVoltage();
    inputs.currentAmpsBottom = new double[] {bottomShooterMotor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    topShooterMotor.setVoltage(volts);
    bottomShooterMotor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    topShooterPID.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
    bottomShooterPID.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    topShooterMotor.stopMotor();
    bottomShooterMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    topShooterPID.setP(kP, 0);
    topShooterPID.setI(kI, 0);
    topShooterPID.setD(kD, 0);
    topShooterPID.setFF(0, 0);

    bottomShooterPID.setP(kP, 0);
    bottomShooterPID.setI(kI, 0);
    bottomShooterPID.setD(kD, 0);
    bottomShooterPID.setFF(0, 0);
  }
}
