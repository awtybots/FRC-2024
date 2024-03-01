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

    // 2x8 matrix of note positions (x,y)
    public static final double[][] notePositions = {
      // Close row
      {2.90, 6.99},
      {2.90, 5.55},
      {2.90, 4.11},
      // Far row
      {8.30, 7.44},
      {8.30, 5.78},
      {8.30, 4.10},
      {8.30, 2.43},
      {8.30, 0.77}
    };
    // Close row
    // notePositions[0][0] = 2.90; // x
    // notePositions[1][0] = 6.99; // y

    // notePositions[0][1] = 2.90; // x
    // notePositions[1][1] = 5.55; // y

    // notePositions[0][2] = 2.90; // x
    // notePositions[1][2] = 4.11; // y

    // // Far row
    // notePositions[0][3] = 8.30; // x
    // notePositions[1][3] = 7.44; // y

    // notePositions[0][4] = 8.30; // x
    // notePositions[1][4] = 5.78; // y

    // notePositions[0][5] = 8.30; // x
    // notePositions[1][5] = 4.10; // y

    // notePositions[0][6] = 8.30; // x
    // notePositions[1][6] = 2.43; // y

    // notePositions[0][7] = 8.30; // x
    // notePositions[1][7] = 0.77; // y
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
    public static final int kBottomFlywheelSparkMaxCanId = 6;

    // TODO implement the below
    public static final int kCurrentLimit = 30;

    public static final int startingAngle = 0;

    public static final double armConversion = 1;

    // Flywheel PID constants
    public static final double kP = 0.03;
    public static final double kI = 0.0;
    public static final double kD = 0.001;

    // Flywheel Feedforward characterization constants
    public static final double ks = 0;
    public static final double kv = 0;
    public static final double shootingVelocity = 5000; // revolutions per second
  }

  public static final class IntakeConstants {
    public static final int kIntakeSparkMaxCanId = 7;
    public static final int kFollowerIntakeSparkMaxCanId = 14;

    public static final int kCurrentLimit = 30;

    public static final int startingAngle = 30;

    public static final double armConversion = 1;

    // Intake PID constants
    public static final double kP = 0.0003;
    public static final double kI = 0.0;
    public static final double kD = 0.001;

    // Intake Feedforward characterization constants
    public static final double ks = 0.1;
    public static final double kv = 5;
    public static double percentPower = 0.3;
  }

  public static final class ArmConstants {
    public static final int kRightArmMotorId = 26;
    public static final int kLeftArmMotorId = 9;

    public static final int kCurrentLimit = 30;

    public static final double armConversion = 0.05;

    public static final double kMaxOutput = 0.3;

    // Arm PID constants
    public static final double kP = 1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kWeightBasedFF = 0.0;

    // Arm Feedforward characterization constants
    public static final double ks = 0.10;
    public static final double kv = 0.05;

    public static final double minimumAngle = 0.1;
    public static final double maximumAngle = Math.PI;

    // do not use
    // public static final double kFF = 0.0;
    // public static final double kIz = 0.0;

    // public static final double initialAngle = 0.0; // Not sure what this is for
  }

  // public static final class ArmElevatorConstants {
  //   public static final int kArmElevatorMotorId = 17; // ! Change before testing

  //   public static final int kCurrentLimit = 30;

  //   public static final int initialExtension = 0;
  //   public static double minExtension = 0;
  //   public static double maxExtension = 2.6; // Inches

  //   // public static final double armElevatorConversion = 1;

  //   // Arm Elevator PID constants
  //   public static final double kP = 1;
  //   public static final double kI = 0;
  //   public static final double kD = 0.0;

  //   // Arm Elevator Feedforward characterization constants
  //   public static final double ks = 0;
  //   public static final double kv = 0;
  // }

  public static final class Presets {
    public static final double ArmThreshold = 0.1;
  }

  public static final class WristConstants {
    public static final int kWristMotorId = 25; // ! Change before testing

    public static final int kCurrentLimit = 30;

    public static final int initialAngle = 0; // radians

    public static final int minAngle = -1; // radians
    public static final int maxAngle = 1; // radians

    // Wrist PID constants
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Wrist Feedforward characterization constants
    public static final double ks = 0.10;
    public static final double kv = 0.05;
  }

  public static final class ClimberConstants {
    public static final int kLeftClimberMotorId = 16;
    public static final int kRightClimberMotorId = 15;

    public static final int kCurrentLimit = 30;

    public static final double initialPosition = 0.7;
    public static final double minPosition = 0.0;
    public static final double maxPosition = 0.55;

    public static final double climberConversion = 1;

    // in meters, i.e. that many meters per rotation
    public static final double gearCircumfrence = 0.134032531982;

    // Climber PID constants
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Climber Feedforward characterization constants
    public static final double ks = 0.10;
    public static final double kv = 0.05;
  }

  // public static final class SticksConstants {
  //   // ! Seems like the sticks won't have motors.
  //   // public static final int kLeftStickMotorId = 19;
  //   // public static final int kRightStickMotorId = 18;

  //   public static final int kCurrentLimit = 30;

  //   public static final int initialAngle = 0; // radians

  //   public static final int minAngle = 0; // radians
  //   public static final int maxAngle = 1; // radians

  //   // Sticks PID constants
  //   public static final double kP = 0.1;
  //   public static final double kI = 0.0;
  //   public static final double kD = 0.0;

  //   // Sticks Feedforward characterization constants
  //   // public static final double ks = 0.10;
  //   // public static final double kv = 0.05;
  // }
}
