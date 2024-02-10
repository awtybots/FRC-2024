// Copyright 2016-2024 FRC 5829
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

package frc.robot.util.math;

public class TalonConversions {

  /**
   * @param counts Falcon Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  public static double falconToDegrees(double counts, double gearRatio) {
    return counts * (360.0 / (gearRatio * 2048.0));
  }

  /**
   * @param degrees Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Falcon Counts
   */
  public static double degreesToFalcon(double degrees, double gearRatio) {
    double ticks = degrees / (360.0 / (gearRatio * 2048.0));
    return ticks;
  }

  /**
   * @param velocityCounts Falcon Velocity Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double falconToRPM(double velocityCounts, double gearRatio) {
    double motorRPM = velocityCounts * (600.0 / 2048.0);
    double mechRPM = motorRPM / gearRatio;
    return mechRPM;
  }

  /**
   * @param RPM RPM of mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double RPMToFalcon(double RPM, double gearRatio) {
    double motorRPM = RPM * gearRatio;
    double sensorCounts = motorRPM * (2048.0 / 600.0);
    return sensorCounts;
  }

  /**
   * @param velocitycounts Falcon Velocity Counts
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   */
  public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
    double wheelRPM = falconToRPM(velocitycounts, gearRatio);
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;
  }

  /**
   * @param velocity Velocity MPS
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   */
  public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
    return wheelVelocity;
  }
}
