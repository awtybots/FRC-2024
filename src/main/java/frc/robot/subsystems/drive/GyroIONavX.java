// Copyright 2021-2024 FRC 5829
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

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import java.util.Queue;
import org.littletonrobotics.junction.AutoLogOutput;

/** IO implementation for NavX */
public class GyroIONavX implements GyroIO {

  AHRS ahrs;
  private final Queue<Double> yawPositionQueue;

  public GyroIONavX(boolean phoenixDrive) {

    try {
      ahrs = new AHRS(SPI.Port.kMXP, (byte) Module.ODOMETRY_FREQUENCY);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    ahrs.reset();
    ahrs.resetDisplacement();
    ahrs.zeroYaw();

    yawPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(() -> (double) ahrs.getAngle());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    // inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    // inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    // inputs.odometryYawPositions =
    //     yawPositionQueue.stream()
    //         .map((Double value) -> Rotation2d.fromDegrees(value))
    //         .toArray(Rotation2d[]::new);
    // yawPositionQueue.clear();
    inputs.connected = ahrs.isConnected();
    inputs.calibrating = ahrs.isCalibrating();
    inputs.yawPosition = Rotation2d.fromDegrees(ahrs.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(ahrs.getRawGyroZ());
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);

    yawPositionQueue.clear();
  }

  public void resetRotation() {
    // ahrs.zeroYaw();
    ahrs.setAngleAdjustment(ahrs.getAngle());
  }

  @AutoLogOutput(key = "Test/NavXAngleAdjustment")
  public double getAngleAdjustment() {
    return ahrs.getAngleAdjustment();
  }
}
//test push