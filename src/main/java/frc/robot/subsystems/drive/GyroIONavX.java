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

/** IO implementation for NavX */
public class GyroIONavX implements GyroIO {

  AHRS ahrs;
  private final Queue<Double> yawPositionQueue;

  public GyroIONavX(boolean phoenixDrive) {

    ahrs.reset();
    ahrs.resetDisplacement();
    ahrs.zeroYaw();

    try {
      ahrs = new AHRS(SPI.Port.kMXP, (byte) Module.ODOMETRY_FREQUENCY);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

    yawPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(() -> (double) ahrs.getYaw());
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
    inputs.yawPosition = Rotation2d.fromDegrees(ahrs.getYaw());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(ahrs.getRawGyroZ());
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);

    yawPositionQueue.clear();
  }
}
