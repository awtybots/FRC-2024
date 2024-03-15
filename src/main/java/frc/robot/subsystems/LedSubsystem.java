package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
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





    public LedSubsystem(int LEDPort, int length, Intake intake) {
        this.length = length;
        this.stripLength = (int) (length / 2);
        this.intake = intake;

        this.defaultColor = new int[] {0, 255, 0};

        if (Math.random() > 0.9) {
          this.defaultColor = new int[] {255, 215, 0};
        }

    try {
      m_led = new AddressableLED(LEDPort);
      m_ledBuffer = new AddressableLEDBuffer(length);
      m_led.setLength(m_ledBuffer.getLength());
      m_led.setData(m_ledBuffer);
      m_led.start();
    } catch (Exception e) {
      e.printStackTrace();
    }
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

  @Override
  public void periodic() {
    boolean noteDetected = intake.getConveyerProximity() || intake.getShooterProximity();
    SmartDashboard.putBoolean("Note Detected", noteDetected);
    if (noteDetected) {
      setColor(defaultColor);
    } else if (DriverStation.getAlliance().get() == Alliance.Red) {

      setColor(new int[] {255, 0, 0});
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      setColor(new int[] {0, 0, 255});
    }

    m_led.setData(m_ledBuffer);
  }

}

