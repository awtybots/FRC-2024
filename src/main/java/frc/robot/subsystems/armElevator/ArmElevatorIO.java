// // Copyright 2021-2024 FRC 6328, FRC 5829
// // http://github.com/Mechanical-Advantage
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

// import org.littletonrobotics.junction.AutoLog;

// public interface ArmElevatorIO {
//   @AutoLog
//   public static class ArmElevatorIOInputs {
//     public double positionInches = 0.0;
//     public double velocityMetersPerSec = 0.0;
//     public double targetDistance = 0.0;
//     public double[] currentAmps = new double[] {};
//   }

//   /** Updates the set of loggable inputs. */
//   public default void updateInputs(ArmElevatorIOInputs inputs) {}

//   /** Run open loop at the specified voltage. */
//   public default void setVoltage(double volts) {}

//   /** Stop in open loop. */
//   public default void stop() {}

//   /** Set velocity PID constants. */
//   public default void configurePID(double kP, double kI, double kD) {}

//   /** In inches */
//   public default void setTargetPosition(double distance) {}
// }
