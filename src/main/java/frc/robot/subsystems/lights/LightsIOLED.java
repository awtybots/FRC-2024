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

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.LedCustomAnimations;
import java.io.File;

public class LightsIOLED implements LightsIO {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private final int length;

  private static double ledCount = 0;
  private static final double LED_SPEED = 1;
  private final int stripLength;

  private boolean stopped = false;

  public int[] rgb = new int[3];

  private int[] GREEN_CODE = {0, 255, 0};
  private int[] PURPLE_CODE = {255, 0, 255};
  private int[] YELLOW_CODE = {255, 255, 0};

  private final LedCustomAnimations BootUp;
  private final LedCustomAnimations Transitions;
  private final LedCustomAnimations ConeToCube;
  private final LedCustomAnimations CubeToCone;
  private final LedCustomAnimations VIVELAFRANCE;
  private LedCustomAnimations Greg;
  private LedCustomAnimations IntakeCone;
  private LedCustomAnimations IntakeCube;
  private LedCustomAnimations PlaceCone;
  private LedCustomAnimations PlaceCube;
  private LedCustomAnimations ShootPiece;

  private LedCustomAnimations[] animations;

  /**
   * Constructs a new AprilTagVisionIOLimelight instance.
   *
   * @param identifier The identifier of the Limelight camera.
   */
  public LightsIOLED(int LEDPort, int length) {
    System.out.println("[Init] Creating Lights");
    this.length = length;
    this.stripLength = (int) (length / 2);

    try {
      led = new AddressableLED(LEDPort);
      ledBuffer = new AddressableLEDBuffer(length);
      led.setLength(ledBuffer.getLength());
      led.setData(ledBuffer);
      led.start();
    } catch (Exception e) {
      // TODO is the stack trace really necessary?
      System.out.println(
          "[Error] LEDs not found. See the following stack trace: " + e.getStackTrace());
      stopped = true;
    }

    // Creating all the animation objects
    BootUp = new LedCustomAnimations(led, ledBuffer, "BootUp2", 200, false);

    Transitions = new LedCustomAnimations(led, ledBuffer, "Transitions", 0, true);

    ConeToCube = new LedCustomAnimations(led, ledBuffer, "ConeToCube", 0, false);
    ConeToCube.end(); // TODO why is this here?

    CubeToCone = new LedCustomAnimations(led, ledBuffer, "CubeToCone", 0, false);
    CubeToCone.end();

    VIVELAFRANCE = new LedCustomAnimations(led, ledBuffer, "VIVELAFRANCE", 0, false);
    VIVELAFRANCE.end();

    Greg = new LedCustomAnimations(led, ledBuffer, "GregAmazingAnimation", 0, true);
    IntakeCone = new LedCustomAnimations(led, ledBuffer, "IntakeGreen", 0, true);
    IntakeCube = new LedCustomAnimations(led, ledBuffer, "IntakeGreen", 0, true);

    PlaceCone = new LedCustomAnimations(led, ledBuffer, "PlaceCone", 0, false);
    PlaceCube = new LedCustomAnimations(led, ledBuffer, "PlaceCube", 0, false);

    ShootPiece = new LedCustomAnimations(led, ledBuffer, "ShootPiece", 0, true);

    final int numberOfAnimations =
        new File(Filesystem.getDeployDirectory(), "5829LedAnimations/").listFiles().length;
    animations = new LedCustomAnimations[numberOfAnimations];
  }

  /**
   * Updates the inputs for the LEDs.
   *
   * @param inputs The LightsIOInputs object containing the inputs.
   *     <p>inputs.color == to {366,366,366} if not a solid color
   */
  @Override
  public void updateInputs(LightsIOInputs inputs) {
    inputs.stopped = stopped;
    inputs.pattern = "";
  }

  /**
   * Look I know this violates the design standards for this code but dammit I need to finish this.
   */
  @Override
  public void runPeriodic() {
    led.setData(ledBuffer);
    led.start();
  }

  public void setLed(int i, int[] color) {
    if (i < 120 && i >= 0) {
      ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }
  }

  public void setHoldAnimation(String animationName, boolean value) {
    LedCustomAnimations animation = null;
    for (LedCustomAnimations tmp_anim : animations) {
      if (tmp_anim.getName() == animationName) {
        animation = tmp_anim;
      }
    }
    if (animation == null) return;
    animation.setIsActive(value);
    if (!value) {
      animation.reset();
    }
  }

  public void setAnimation(String animationName, boolean value) {
    LedCustomAnimations animation = null;
    for (LedCustomAnimations tmp_anim : animations) {
      if (tmp_anim.getName() == animationName) {
        animation = tmp_anim;
      }
    }
    if (animation == null) return;

    if (value) {
      animation.reset();
      animation.setLoop(true);
    } else {
      animation.setLoop(false);
      animation.end();
    }
    animation.setIsActive(value);
  }

  @Override
  public void reset() {
    LedCustomAnimations animation = null;
    for (LedCustomAnimations tmp_anim : animations) {
      if (animation != null && tmp_anim.isActive()) {
        System.out.println("[ERROR] Multiple animations somehow active. FIX THIS!");
      } else if (tmp_anim.isActive()) {
        animation = tmp_anim;
      }
    }
    animation.reset();
  }

  @Override
  public boolean isFinished() {
    LedCustomAnimations animation = null;
    for (LedCustomAnimations tmp_anim : animations) {
      if (animation != null && tmp_anim.isActive()) {
        System.out.println("[ERROR] Multiple animations somehow active. FIX THIS!");
      } else if (tmp_anim.isActive()) {
        animation = tmp_anim;
      }
    }
    return animation.isFinished();
  }

  @Override
  public boolean isActive(String animationName) {
    LedCustomAnimations animation = null;
    for (LedCustomAnimations tmp_anim : animations) {
      if (tmp_anim.getName() == animationName) {
        animation = tmp_anim;
      }
    }
    return animation.isActive();
  }

  // public void fillRange(int first, int last, int[] color) {
  //     for (int i = first; i < ledBuffer.getLength() && i < last; i++) {
  //         ledBuffer.setRGB(i, color[0], color[1], color[2]);
  //     }
  // }

  // public void setColor(int c) {
  //     if (c == 0) {
  //         rgb = PURPLE_CODE;
  //     }
  //     if (c == 1) {
  //         rgb = YELLOW_CODE;
  //     }
  // }

  // private void SolidColor() { // TODO Repurpose Cone/Cube code for this year (holding note or
  // not)
  //     if (RobotContainer.getCurrentState() == RobotContainer.State.Shooting) {
  //         ShootPiece.setAnimation();
  //     } else {
  //         ShootPiece.reset();
  //         if (RobotContainer.getIsCone()) {
  //             ConeToCube.reset();
  //             CubeToCone.setAnimation();
  //             if (CubeToCone.isFinished()) {
  //                 if (IntakeCone.isActive()) {
  //                     System.out.println("INTAKE CONE IS ACTIVE");
  //                     IntakeCone.setAnimation();
  //                 } else {
  //                     for (int i = 0; i < ledBuffer.getLength(); i++) {
  //                         setLed(i, YELLOW_CODE);
  //                     }
  //                 }
  //             }
  //         } else {
  //             CubeToCone.reset();
  //             ConeToCube.setAnimation();
  //             PlaceCube.reset();
  //             if (ConeToCube.isFinished()) {
  //                 if (IntakeCube.isActive()) {
  //                     System.out.println("INTAKE CUBE IS ACTIVE");
  //                     IntakeCube.setAnimation();
  //                 } else {
  //                     for (int i = 0; i < ledBuffer.getLength(); i++) {
  //                         setLed(i, PURPLE_CODE);
  //                     }
  //                 }
  //             }
  //         }
  //     }
  // }

  // public void resetAnimations(){
  //     for (LedCustomAnimations anim : animations) {
  //         anim.end();
  //         anim.setLoop(false);
  //     }
  // }

  /** Method to turn on a certain percentage of the lights of a LED strip to a certain color. */
  // public void turnOn(double portionLED, int r, int g, int b, int r2, int g2, int b2) {
  //     assert portionLED <= 1;
  //     for (var i = 0; i <= ledBuffer.getLength(); i++) {
  //         if (i <= ledBuffer.getLength() * portionLED) {
  //             ledBuffer.setRGB(i, r, g, b);
  //         } else {
  //             ledBuffer.setRGB(i, r2, g2, b2);
  //         }
  //     }
  //     led.setData(ledBuffer);
  // }

  /**
   * Method to change the amount of lights a LED strip has as green based on the area of the screen
   * that a target takes up in the Limelight Apriltag detection system.
   */
  // public void visionTrackingLED(double area) {
  //     for (var i = 0; i < ledBuffer.getLength(); i++) {
  //         if (area > 17) {
  //             ledBuffer.setRGB(i, 0, 200, 0);
  //         } else if (area * 10 > i) {
  //             ledBuffer.setRGB(i, 0, 0, 200);
  //         } else {
  //             ledBuffer.setRGB(i, 100, 100, 100);
  //         }
  //     }
  //     led.setData(ledBuffer);
  // }

  // public void turnOff() {
  //     for (var i = 0; i < ledBuffer.getLength(); i++) {
  //         ledBuffer.setRGB(i, 0, 0, 0);
  //     }
  //     led.setData(ledBuffer);
  // }

  // private void Animations() {
  //     for (int i = 0; i < ledBuffer.getLength(); i++) {
  //         setLed(i, GREEN_CODE);
  //     }

  //     if (RobotContainer.getIsCone()) setColor(1);
  //     else setColor(0);
  //     ledCount += LED_SPEED;
  //     for (int i = 0; i < stripLength; i++) {
  //         setLed(((int) (i + ledCount) % length), rgb);
  //     }

  //     if (ledCount > length) {
  //         ledCount = 0;
  //     }
  // }

  // private void GayRainbow(int offset) {
  //     final int hueShiftRate = 20;
  //     int hue = offset % 360;
  //     for (int i = 0; i < ledBuffer.getLength(); i++) {
  //         ledBuffer.setHSV(i, hue, 80, 80);
  //         hue = (hue + hueShiftRate) % 360;
  //     }
  // }

  // private int offset = 0;

  // private void RotatingRainbow() {
  //     final int speed = 2; // TODO tune led speed
  //     offset = (offset + speed) % ledBuffer.getLength();
  //     GayRainbow(offset);
  // }

  // boolean rainbowMode = true;
}
