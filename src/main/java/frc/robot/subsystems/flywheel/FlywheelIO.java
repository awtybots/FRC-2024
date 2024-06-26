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

import org.littletonrobotics.junction.AutoLog;

/** Interface for operative components of Flywheel (shooter) subsystem. */
public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public double positionRadTop = 0.0;
    public double positionRadBottom = 0.0;
    public double velocityRadPerSecTop = 0.0;
    public double velocityRadPerSecBottom = 0.0;
    public double appliedVoltsTop = 0.0;
    public double appliedVoltsBottom = 0.0;
    public double[] currentAmpsTop = new double[] {};
    public double[] currentAmpsBottom = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
