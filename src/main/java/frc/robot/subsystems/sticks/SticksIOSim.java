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

// package frc.robot.subsystems.sticks;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import frc.robot.Constants.SticksConstants;

// public class SticksIOSim implements SticksIO {
//   // ! The settings on this simulation are wrong, fix later (or not lol)
//   private SingleJointedArmSim sim =
//       new SingleJointedArmSim(DCMotor.getNEO(1), 1.5, 0.004, 1, 0, 1.047, true, 0.524);
//   private PIDController pid =
//       new PIDController(SticksConstants.kP, SticksConstants.kI, SticksConstants.kD);

//   private boolean closedLoop = false;
//   private double ffVolts = 0.0;
//   private double appliedVolts = 0.0;

//   @Override
//   public void updateInputs(SticksIOInputs inputs) {
//     if (closedLoop) {
//       appliedVolts =
//           MathUtil.clamp(pid.calculate(sim.getVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
//       sim.setInputVoltage(appliedVolts);
//     }

//     sim.update(0.02);

//     inputs.positionRad = 0.0;
//     inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
//     inputs.appliedVolts = appliedVolts;
//     inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
//   }

//   @Override
//   public void setVoltage(double volts) {
//     closedLoop = false;
//     appliedVolts = 0.0;
//     sim.setInputVoltage(volts);
//   }

//   // @Override
//   // public void setVelocity(double velocityRadPerSec, double ffVolts) {
//   //   closedLoop = true;
//   //   pid.setSetpoint(velocityRadPerSec);
//   //   this.ffVolts = ffVolts;
//   // }

//   @Override
//   public void stop() {
//     setVoltage(0.0);
//   }

//   @Override
//   public void configurePID(double kP, double kI, double kD) {
//     pid.setPID(kP, kI, kD);
//   }
// }
