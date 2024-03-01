// Copyright 2021-2024 FRC 6328
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

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.DriveConstants;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  // Gear ratios for SDS MK4i L2, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = 5.08 / 1.0;
  private static final double TURN_GEAR_RATIO = 1.0; // 46.42/1.0;
  private static final double RELATIVE_ENCODER_TURN_GEAR_RATIO = 46.42 / 1.0;

  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AnalogInput turnAbsoluteEncoder;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final SparkAbsoluteEncoder turnAbsoluteEncoderNew;

  private final boolean isTurnMotorInverted = false;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
        // Front left
      case 0:
        driveSparkMax =
            new CANSparkMax(DriveConstants.kFrontLeftDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(DriveConstants.kFrontLeftTurningCanId, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(0);
        absoluteEncoderOffset = new Rotation2d(Math.PI / 2.0 + Math.PI); // MUST BE CALIBRATED
        break;
        // Front right
      case 1:
        driveSparkMax =
            new CANSparkMax(DriveConstants.kFrontRightDrivingCanId, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(DriveConstants.kFrontRightTurningCanId, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(1);
        absoluteEncoderOffset = new Rotation2d(0 + Math.PI + Math.PI); // MUST BE CALIBRATED
        break;
        // Rear left
      case 2:
        driveSparkMax = new CANSparkMax(DriveConstants.kRearLeftDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(DriveConstants.kRearLeftTurningCanId, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(2);
        absoluteEncoderOffset =
            new Rotation2d(
                Math.PI + Math.PI + Math.PI + Math.PI / 8 + Math.PI / 16); // MUST BE CALIBRATED
        break;
        // Rear right
      case 3:
        driveSparkMax =
            new CANSparkMax(DriveConstants.kRearRightDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(DriveConstants.kRearRightTurningCanId, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(3);
        absoluteEncoderOffset =
            new Rotation2d(
                Math.PI * 3 / 2 + Math.PI + Math.PI + Math.PI + Math.PI / 32); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnAbsoluteEncoderNew = turnSparkMax.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    turnAbsoluteEncoderNew.setInverted(true);

    turnSparkMax.setInverted(isTurnMotorInverted);
    driveSparkMax.setSmartCurrentLimit(20);
    turnSparkMax.setSmartCurrentLimit(20);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    turnSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    // turnPositionQueue =
    //   SparkMaxOdometryThread.getInstance().registerSignal(turnRelativeEncoder::getPosition);

    turnPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(turnAbsoluteEncoderNew::getPosition);
    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    // inputs.turnAbsolutePosition =
    //     new Rotation2d(
    //             turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 *
    // Math.PI)
    //         .minus(absoluteEncoderOffset);

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsoluteEncoderNew.getPosition() / TURN_GEAR_RATIO)
            .minus(absoluteEncoderOffset);

    // New code to get the absolute encoder with the CTRE through bore absolute encoder

    inputs.turnPosition =
        Rotation2d.fromRotations(turnAbsoluteEncoderNew.getPosition() / TURN_GEAR_RATIO)
            .minus(absoluteEncoderOffset);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnAbsoluteEncoderNew.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map(
                (Double value) ->
                    Rotation2d.fromRotations(value / TURN_GEAR_RATIO).minus(absoluteEncoderOffset))
            .toArray(Rotation2d[]::new);
    drivePositionQueue.clear();
    turnPositionQueue.clear();
    turnRelativeEncoder.setPosition(turnAbsoluteEncoderNew.getPosition());
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
