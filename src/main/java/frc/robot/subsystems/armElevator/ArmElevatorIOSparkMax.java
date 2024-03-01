// // Copyright 2021-2024 FRC 6328, FRC 5829
// //
// // This program is free software; you can redistribute it and/or
// // modify it under the terms of the GNU General Public License
// // version 3 as published by the Free Software Foundation or
// // available in the root directory of this project.
// //
// // This program is distributed in the hope that it will be useful,
// // but WITHOUT ANY WARRANTY; without even the implied warranty of
// // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// // GNU General Public License for more details.

// package frc.robot.subsystems.armElevator;

// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.SparkPIDController.ArbFFUnits;
// import edu.wpi.first.math.MathUtil;
// import frc.robot.Constants.ArmElevatorConstants;

// /**
//  * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
//  * "CANSparkFlex".
//  */
// public class ArmElevatorIOSparkMax implements ArmElevatorIO {
//   private static final double GEAR_RATIO = 40.0; // gear ratio
//   private static final double PULLEY_DIAMETER = 2.0; // pulley diameter in meters
//   private static final double PULLEY_CIRCUMFERENCE =
//       Math.PI * PULLEY_DIAMETER; // pulley circumference

//   private double targetDistance = 0.0;

//   private final CANSparkMax motor =
//       new CANSparkMax(ArmElevatorConstants.kArmElevatorMotorId, MotorType.kBrushless);
//   private final RelativeEncoder encoder = motor.getEncoder();
//   private final SparkPIDController pid = motor.getPIDController();

//   public ArmElevatorIOSparkMax() {
//     motor.restoreFactoryDefaults();

//     motor.setCANTimeout(250);

//     motor.setInverted(true);

//     motor.setSmartCurrentLimit(ArmElevatorConstants.kCurrentLimit);

//     // pid.setOutputRange(0, ArmElevatorConstants.maxExtension);

//     // motor.burnFlash();
//   }

//   @Override
//   public void updateInputs(ArmElevatorIOInputs inputs) {
//     inputs.positionInches = encoder.getPosition() / GEAR_RATIO * PULLEY_CIRCUMFERENCE;
//     inputs.velocityMetersPerSec = encoder.getVelocity() / GEAR_RATIO * PULLEY_CIRCUMFERENCE;
//     inputs.targetDistance = targetDistance;
//     inputs.currentAmps = new double[] {motor.getOutputCurrent()};
//   }

//   @Override
//   public void setVoltage(double volts) {
//     motor.setVoltage(volts);
//   }

//   // @Override
//   // public void setVelocity(double velocityRadPerSec, double ffVolts) {
//   // pid.setReference(
//   //     Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
//   //     ControlType.kVelocity,
//   //     0,
//   //     ffVolts,
//   //     ArbFFUnits.kVoltage);
//   // }

//   @Override
//   public void stop() {
//     motor.stopMotor();
//   }

//   @Override
//   public void configurePID(double kP, double kI, double kD) {
//     pid.setP(kP, 0);
//     pid.setI(kI, 0);
//     pid.setD(kD, 0);
//     pid.setFF(0, 0);
//   }

//   @Override
//   public void setTargetPosition(double distanceInches) {
//     targetDistance =
//         MathUtil.clamp(
//             distanceInches, ArmElevatorConstants.minExtension,
// ArmElevatorConstants.maxExtension);
//     pid.setReference(
//         targetDistance / PULLEY_CIRCUMFERENCE * GEAR_RATIO,
//         ControlType.kPosition,
//         0,
//         0,
//         ArbFFUnits.kVoltage);
//   }
// }
