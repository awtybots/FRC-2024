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
  public static final class EnvironmentalConstants {
    /*
    ! Change this when testing btwn modes yourself. It's technically possible to connect this to a LoggedDashboardChooser for
    ! convinence, but we shouldn't do that as that would mess with the REPLAY mode.
    */
    public static final Mode currentMode = Mode.REAL;
  }

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
    public static final int kFrontRightDrivingCanId = 12;
    public static final int kRearRightDrivingCanId = 2;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 4;
    public static final int kFrontRightTurningCanId = 13;
    public static final int kRearRightTurningCanId = 5;
  }

  public static final class FlywheelConstants {

    // CAN ID's
    public static final int kTopFlywheelSparkMaxCanId = 3;
    public static final int kBottomFlywheelSparkMaxCanId = 14;

    // TODO implement the below
    public static final int kCurrentLimit = 30;

    public static final int startingAngle = 0; // TODO TUNE

    public static final double armConversion = 1; // TODO TUNE

    // Arm PID constants
    public static final double kP = 0.0004; // TODO TUNE
    public static final double kI = 0.0; // TODO TUNE
    public static final double kD = 0.01; // TODO TUNE

    // Arm Feedforward characterization constants
    public static final double ks = 0; // TODO TUNE
    public static final double kv = 0.4; // TODO TUNE
  }

  public static final class IntakeConstants {
    public static final int kTopIntakeSparkMaxCanId = 7; // ! Change before testing
    public static final int kBottomIntakeSparkMaxCanId = 101;

    public static final int kCurrentLimit = 30;

    public static final int startingAngle = 30; // TODO TUNE

    public static final double armConversion = 1; // TODO TUNE

    public static final double kP = 0.0007; // TODO TUNE
    public static final double kI = 0.0; // TODO TUNE
    public static final double kD = 0.001; // TODO TUNE

    // Arm Feedforward characterization constants
    public static final double ks = 0.1; // TODO TUNE
    public static final double kv = 0.9; // TODO TUNE
    public static double velocity = 300;
  }

  public static final class ArmConstants {
    public static final int kRightArmMotorId = 1;
    public static final int kLeftArmMotorId = 9;

    public static final int kCurrentLimit = 30;

    public static final double armConversion = 0.05; // TODO TUNE

    // Arm PID constants
    public static final double kP = 0.1; // TODO TUNE
    public static final double kI = 0.0; // TODO TUNE
    public static final double kD = 0.0; // TODO TUNE

    // Arm Feedforward characterization constants
    public static final double ks = 0.10; // TODO TUNE
    public static final double kv = 0.05; // TODO TUNE

    public static final double minimumAngle = -1.441;
    public static final double maximumAngle = 1.441;

    // do not use
    // public static final double kFF = 0.0;
    // public static final double kIz = 0.0;

    // public static final double initialAngle = 0.0; // Not sure what this is for
  }

  public static final class ArmElevatorConstants {
    public static final int kArmElevatorMotorId = 17; // ! Change before testing

    public static final int kCurrentLimit = 30;

    public static final int initialExtension = 0; // TODO TUNE
    public static final int maxExtension = 1; // TODO TUNE

    public static final double armElevatorConversion = 1; // TODO TUNE

    // ArmElevator PID constants
    public static final double kP = 0.1; // TODO TUNE
    public static final double kI = 0.0; // TODO TUNE
    public static final double kD = 0.0; // TODO TUNE

    // Arm Elevator Feedforward characterization constants
    public static final double ks = 0.10; // TODO TUNE
    public static final double kv = 0.05; // TODO TUNE

    // public static final double ArmThreshold = 0.1; // TODO TUNE

  }

  public static final class Presets {
    public static final double ArmThreshold = 0.1; // TODO TUNE
  }

  public static final class WristConstants {
    public static final int kWristMotorId = 6; // ! Change before testing

    public static final int kCurrentLimit = 30;

    public static final int initialExtension = 0; // TODO TUNE
    public static final int maxExtension = 1; // TODO TUNE

    public static final double armElevatorConversion = 1; // TODO TUNE

    // ArmElevator PID constants
    public static final double kP = 0.1; // TODO TUNE
    public static final double kI = 0.0; // TODO TUNE
    public static final double kD = 0.0; // TODO TUNE

    // Arm Elevator Feedforward characterization constants
    public static final double ks = 0.10; // TODO TUNE
    public static final double kv = 0.05; // TODO TUNE
  }

  public static final class ClimberConstants {
    public static final int kLeftClimberMotorId = 15; // ! Change before testing
    public static final int kRightClimberMotorId = 16; // ! Change before testing

    public static final int kCurrentLimit = 30;

    public static final int initialExtension = 0; // TODO TUNE
    public static final int maxExtension = 1; // TODO TUNE

    public static final double armElevatorConversion = 1; // TODO TUNE

    // ArmElevator PID constants
    public static final double kP = 0.1; // TODO TUNE
    public static final double kI = 0.0; // TODO TUNE
    public static final double kD = 0.0; // TODO TUNE

    // Arm Elevator Feedforward characterization constants
    public static final double ks = 0.10; // TODO TUNE
    public static final double kv = 0.05; // TODO TUNE
  }
}
