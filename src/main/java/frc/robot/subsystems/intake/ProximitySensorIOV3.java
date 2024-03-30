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

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.IntakeConstants;

/** IO implementation for a proximity sensor V3 */
public class ProximitySensorIOV3 implements ProximitySensorIO {

  private DigitalInput conveyorSensor;
  private DigitalInput shooterSensor;

  public ProximitySensorIOV3() {
    try {
      conveyorSensor = new DigitalInput(IntakeConstants.conveyorSensor);
    } catch (RuntimeException ex) {
      DriverStation.reportError(
          "Error instantiating DigitalInput conveyorSensor: " + ex.getMessage(), true);
    }
    try {
      shooterSensor = new DigitalInput(IntakeConstants.shooterSensor);
    } catch (RuntimeException ex) {
      DriverStation.reportError(
          "Error instantiating DigitalInput shooterSensor: " + ex.getMessage(), true);
    }
  }

  @Override
  public void updateInputs(ProximitySensorIOInputs inputs) {
    inputs.isConveyorSensorTriggered = !conveyorSensor.get();
    inputs.isShooterSensorTriggered = !shooterSensor.get();
  }
}
