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

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DriveConstants {
    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 9;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 5;
  }

  public static final class FlywheelConstants {

    // CAN ID's
    public static final int kTopFlywheelSparkMaxCanId = 13;
    public static final int kBottomFlywheelSparkMaxCanId = 14;
  }

  public static final class Arm{
    public static final int kRightArmMotorId = 15;
    public static final int kLeftArmMotorId = 16;
    
    public static final int kCurrentLimit = 30;

    public static final int startingAngle = 30; // TODO TUNE

    public static final double armConversion = 1; //TODO TUNE

    public static final double kP = 0.1; //TODO TUNE
    public static final double kI = 0.0; //TODO TUNE
    public static final double kD = 0.0; //TODO TUNE
    public static final double kFF = 0.0; //TODO TUNE
    public static final double kIz = 0.0; //TODO TUNE

    public static final double initialHeight = 0.0; //TODO TUNE

  }

  public static final class Presets{
    public static final double ArmThreshold = 0.1; //TODO TUNE

  }
}
