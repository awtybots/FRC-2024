// Copyright 2016-2024 FRC 5289
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

import org.littletonrobotics.junction.AutoLog;

public interface ProximitySensorIO {
  @AutoLog
  public static class ProximitySensorIOInputs {
    public boolean isConveyorSensorTriggered;
    public boolean isShooterSensorTriggered;
  }

  public default void updateInputs(ProximitySensorIOInputs inputs) {}
}
