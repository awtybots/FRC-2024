package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ResetGyro {

  public static Command resetGyro() {
    return new Command() {
      @Override
      public void initialize() {}

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }
}
