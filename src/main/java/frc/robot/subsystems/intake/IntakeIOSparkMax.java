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

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO = 52.0 / 34.0; // May be reciprocal

  private final CANSparkMax intakeMotor =
      new CANSparkMax(IntakeConstants.kTopIntakeSparkMaxCanId, MotorType.kBrushless);

 private final CANSparkMax followerMotor =
      new CANSparkMax(IntakeConstants.kBottomIntakeSparkMaxCanId, MotorType.kBrushless);



  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  private final SparkPIDController intakePID = intakeMotor.getPIDController();

  public IntakeIOSparkMax() {
    intakeMotor.restoreFactoryDefaults();
    followerMotor.restoreFactoryDefaults();

    intakeMotor.setCANTimeout(250);
    followerMotor.setCANTimeout(250);

    intakeMotor.setInverted(false);
    followerMotor.setInverted(false);


    intakeMotor.enableVoltageCompensation(12.0);
    intakeMotor.setSmartCurrentLimit(30);
     followerMotor.enableVoltageCompensation(12.0);
    followerMotor.setSmartCurrentLimit(30);

    intakeMotor.burnFlash();
        followerMotor.burnFlash();

    followerMotor.follow(intakeMotor);

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(intakeEncoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(intakeEncoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.currentAmps = new double[] {intakeMotor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    intakePID.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    intakeMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    intakePID.setP(kP, 0);
    intakePID.setI(kI, 0);
    intakePID.setD(kD, 0);
    intakePID.setFF(0, 0);
  }
}
