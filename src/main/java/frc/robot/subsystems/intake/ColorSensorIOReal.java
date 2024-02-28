// Copyright 2024 FRC 5829
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

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

/** IO implementation for NavX */
public class ColorSensorIOReal implements ColorSensorIO {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 colorSensor;

  public ColorSensorIOReal() {
    try {
      colorSensor = new ColorSensorV3(i2cPort);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating ColorSensorV3:  " + ex.getMessage(), true);
    }
  }

  @Override
  public void updateInputs(ColorSensorIOInputs inputs) {
    inputs.connected = colorSensor.isConnected();
    inputs.red = colorSensor.getRed();
    inputs.blue = colorSensor.getBlue();
    inputs.green = colorSensor.getGreen();
    inputs.IR = colorSensor.getIR();
    inputs.proximity = colorSensor.getProximity();
  }
}
