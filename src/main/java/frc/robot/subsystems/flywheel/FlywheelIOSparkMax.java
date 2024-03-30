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

package frc.robot.subsystems.flywheel;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.Constants.FlywheelConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class FlywheelIOSparkMax implements FlywheelIO {
  private static final double GEAR_RATIO = 2;

  /*
   Note: I do not believe there will be any situation where the top motor and the bottom motor
   will need to be treated differently (because if they were, they could rip the note apart in
   the current design). The following could also be achieved with a follower motor.
  */
  private final CANSparkMax topShooterMotor =
      new CANSparkMax(FlywheelConstants.kTopFlywheelSparkMaxCanId, MotorType.kBrushless);
  private final RelativeEncoder topShooterEncoder = topShooterMotor.getEncoder();
  private final SparkPIDController topShooterPID = topShooterMotor.getPIDController();
  private final PIDController topMotorPID;

  private final CANSparkMax bottomShooterMotor =
      new CANSparkMax(FlywheelConstants.kBottomFlywheelSparkMaxCanId, MotorType.kBrushless);
  private final RelativeEncoder bottomShooterEncoder = bottomShooterMotor.getEncoder();
  private final SparkPIDController bottomShooterPID = bottomShooterMotor.getPIDController();
  private final PIDController bottomMotorPID;

  private double targetVelocity;

  public FlywheelIOSparkMax() {
    topShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.restoreFactoryDefaults();

    topShooterMotor.setCANTimeout(250);
    bottomShooterMotor.setCANTimeout(250);

    topShooterMotor.setInverted(true);
    bottomShooterMotor.setInverted(true);

    topShooterMotor.setSmartCurrentLimit(30);
    bottomShooterMotor.setSmartCurrentLimit(30);

    topMotorPID =
        new PIDController(
            Constants.FlywheelConstants.kP,
            Constants.FlywheelConstants.kI,
            Constants.FlywheelConstants.kD);
    bottomMotorPID =
        new PIDController(
            Constants.FlywheelConstants.kP,
            Constants.FlywheelConstants.kI,
            Constants.FlywheelConstants.kD);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.positionRadTop = topShooterEncoder.getPosition();
    inputs.velocityRadPerSecTop = topShooterEncoder.getVelocity();
    inputs.appliedVoltsTop = topShooterMotor.getAppliedOutput() * topShooterMotor.getBusVoltage();
    inputs.currentAmpsTop = new double[] {topShooterMotor.getOutputCurrent()};

    inputs.positionRadBottom = bottomShooterEncoder.getPosition();
    inputs.velocityRadPerSecBottom = bottomShooterEncoder.getVelocity();
    inputs.appliedVoltsBottom =
        bottomShooterMotor.getAppliedOutput() * bottomShooterMotor.getBusVoltage();
    inputs.currentAmpsBottom = new double[] {bottomShooterMotor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    topShooterMotor.setVoltage(volts);
    bottomShooterMotor.setVoltage(volts);
  }

  @AutoLogOutput(key = "Flywheel/targetVelocity")
  public double getTargetVelocity() {
    return targetVelocity;
  }

  @AutoLogOutput(key = "Flywheel/TopVelocity")
  public double getTopVelocity() {
    return topShooterEncoder.getVelocity();
  }

  @AutoLogOutput(key = "Flywheel/BottomVelocity")
  public double getBottomVelocity() {
    return bottomShooterEncoder.getVelocity();
  }

  @Override
  public void setVelocity(double velocityRevPerSec, double ffVolts) { // DO NOT TRUST UNITS
    targetVelocity = velocityRevPerSec;
    topShooterPID.setReference(targetVelocity, ControlType.kVelocity, 0, ffVolts);
    bottomShooterPID.setReference(targetVelocity, ControlType.kVelocity, 0, ffVolts);
    double topShooterVelocity = topShooterEncoder.getVelocity();
    double bottomShooterVelocity = topShooterEncoder.getVelocity();

    topShooterMotor.set(
        topMotorPID.calculate(topShooterVelocity, velocityRevPerSec)
            + Constants.FlywheelConstants.kv * velocityRevPerSec
            + Constants.FlywheelConstants.ks);

    bottomShooterMotor.set(
        bottomMotorPID.calculate(bottomShooterVelocity, velocityRevPerSec)
            + Constants.FlywheelConstants.kv * velocityRevPerSec
            + Constants.FlywheelConstants.ks);
  }

  @Override
  public void stop() {
    topShooterMotor.stopMotor();
    bottomShooterMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    topMotorPID.setP(kP);
    topMotorPID.setI(kI);
    topMotorPID.setD(kD);

    bottomMotorPID.setP(kP);
    bottomMotorPID.setI(kI);
    bottomMotorPID.setD(kD);
  }
}
