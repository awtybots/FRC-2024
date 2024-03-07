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

import org.littletonrobotics.junction.AutoLog;

public interface LightsIO {

  @AutoLog
  public static class LightsIOInputs {
    public boolean stopped = false;
    public String pattern = "";
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LightsIOInputs inputs) {}

  /**
   * Run a specific animation.
   *
   * @param animation The name of the animation.
   */
  public default void setAnimation(String animation) {}

  /**
   * Set LEDs to solid color.
   *
   * @param color The color to set it to. int[3], where {r,g,b}
   */
  public default void setColor(int[] color) {}

  /** Turn the LEDs off. */
  public default void stop() {}

  /** Stop the animation of the LEDs, and reset to a neutral color. */
  // TODO this a good idea?
  public default void stopAnimation() {}

  public default void reset() {}

  public default boolean isActive(String animationName) {
    return false;
  }

  public default boolean isFinished() {
    return false;
  }

  public default void runPeriodic() {}
}
