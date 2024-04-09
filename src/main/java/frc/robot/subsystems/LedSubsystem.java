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
/*
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;

public class LedSubsystem extends SubsystemBase {

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private final int length;

  private static double ledCount = 0;
  private static final double LED_SPEED = 1;
  private final int stripLength;


  Intake intake;
  private int[] defaultColor;
  private boolean isLucky;

  public LedSubsystem(int LEDPort, int length, Intake intake) {
    this.length = length;
    this.stripLength = (int) (length / 2);
    this.intake = intake;

    defaultColor = new int[] {0, 255, 0};

    isLucky = Math.random() > 0.99999;




  }

  public void setLed(int i, int[] color) {
    if (i < 120 && i >= 0) {
      m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }
  }

  public void setColor(int[] color) {
    for (int i = 0; i < length; i++) {
      m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }
  }

  public void setDisco() {
    for (int i = 0; i < length; i++) {
      m_ledBuffer.setRGB(
          i, (int) Math.random() * 255, (int) Math.random() * 255, (int) Math.random() * 255);
    }
  }

  @Override
  public void periodic() {
    boolean noteDetected = intake.getConveyerProximity() || intake.getShooterProximity();
    SmartDashboard.putBoolean("Note Detected", noteDetected);
    if (noteDetected) {
      setColor(new int[] {0, 255, 0});
    } else if (isLucky) {
      setColor(new int[] {255, 215, 0});

    } else if (DriverStation.getAlliance().get() == Alliance.Red) {

      setColor(new int[] {255, 0, 0});
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      setColor(new int[] {0, 0, 255});
    }

    m_led.setData(m_ledBuffer);
  }
}
*/
