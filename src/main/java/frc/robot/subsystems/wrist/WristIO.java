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

// package frc.robot.subsystems.wrist;
// ! Note: if you reimplement this, make sure to implement changes from upstream.

// import frc.robot.Constants.WristConstants;
// import org.littletonrobotics.junction.AutoLog;

// public interface WristIO {
//   @AutoLog
//   public static class WristIOInputs {
//     public double positionRad = 0.0;
//     public double velocityRadPerSec = 0.0;
//     public double appliedVolts = 0.0;
//     public double[] currentAmps = new double[] {};
//     public double targetPositionRad = WristConstants.initialAngle;
//   }

//   /** Updates the set of loggable inputs. */
//   public default void updateInputs(WristIOInputs inputs) {}

//   /** Run open loop at the specified voltage. */
//   public default void setVoltage(double volts) {}

//   /** Run closed loop at the specified velocity. */
//   public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

//   /** Run closed loop for the specified target angle. */
//   public default void setTargetAngle(double angle) {}

//   /** Stop in open loop. */
//   public default void stop() {}

//   /** Set velocity PID constants. */
//   public default void configurePID(double kP, double kI, double kD) {}
// }
