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

package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Lights extends SubsystemBase {

  private final LightsIO io;
  private final LightsIOInputsAutoLogged inputs = new LightsIOInputsAutoLogged();
  private final int[] defaultTeleopColor = {0, 255, 0};
  private final int[] defaultAutonomousColor = {0, 200, 0};

  public Lights(LightsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    if (inputs.stopped) return;
    // TODO Use New Custom Animation Software
    if (io.isActive("VIVELAFRANCE")) {
      io.setAnimation("VIVELAFRANCE");
    } else {
      // if (RobotContainer.getIsCone()
      //     && (RobotContainer.getCurrentState() == State.HighNode
      //         || RobotContainer.getCurrentState() == State.MidNode)) {
      //   PlaceCube.reset();
      //   PlaceCone.setAnimation();
      //   if (PlaceCone.isFinished()) {
      //     for (int i = 0; i < ledBuffer.getLength(); i++) {
      //       setLed(i, YELLOW_CODE);
      //     }
      //   }
      // } else if (!RobotContainer.getIsCone()
      //     && (RobotContainer.getCurrentState() == State.HighNode
      //         || RobotContainer.getCurrentState() == State.MidNode)) {
      //   PlaceCone.reset();
      //   PlaceCube.setAnimation();
      //   if (PlaceCube.isFinished()) {
      //     for (int i = 0; i < ledBuffer.getLength(); i++) {
      //       setLed(i, PURPLE_CODE);
      //     }
      //   }
      // } else {
      io.reset();
      if (DriverStation.isTeleopEnabled()) {
        io.setColor(defaultTeleopColor);
      } else {
        io.setAnimation("BootUp");
        if (io.isFinished()) {
          if (DriverStation
              .isAutonomousEnabled()) { // TODO reimplement the state logic in RobotContainer and
            // here
            // && RobotContainer.getCurrentState() != State.Stow) {
            io.setColor(defaultAutonomousColor);
          } else {
            io.setAnimation("Transitions");
          }
        }
      }
    }
    io.runPeriodic();
    io.updateInputs(inputs);
    Logger.processInputs("Lights", inputs);
  }

  /**
   * Run a specific animation.
   *
   * @param animation The name of the animation.
   */
  public void setAnimation(String animation) {
    io.setAnimation(animation);
  }

  /**
   * Set LEDs to solid color.
   *
   * @param color The color to set it to. int[3], where {r,g,b}
   */
  public void setColor(int[] color) {
    io.setColor(color);
  }

  /** Stop the animation of the LEDs, and reset to a neutral color. */
  // TODO this a good idea?
  public void stopAnimation() {
    io.stopAnimation();
  }

  /** Turn the LEDs off. */
  public void stop() {
    io.stop();
  }
}
