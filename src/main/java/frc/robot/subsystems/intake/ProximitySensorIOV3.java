// Copyright (c) 2024 FRC 5829
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
import frc.robot.Constants.IntakeConstants;

/** IO implementation for a proximity sensor V3 */
public class ProximitySensorIOV3 implements ProximitySensorIO {

  private DigitalInput conveyorSensor;
  private DigitalInput shooterSensor;

  public ProximitySensorIOV3() {
    conveyorSensor = new DigitalInput(IntakeConstants.conveyorSensor);
    shooterSensor = new DigitalInput(IntakeConstants.shooterSensor);
  }

  @Override
  public void updateInputs(ProximitySensorIOInputs inputs) {
    inputs.isConveyorSensorTriggered = !conveyorSensor.get();
    inputs.isShooterSensorTriggered = !shooterSensor.get();
  }
}
