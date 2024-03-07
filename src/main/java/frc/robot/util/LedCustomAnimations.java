package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.FileReader;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class LedCustomAnimations {

  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;
  private boolean isLoop;
  private boolean isActive;
  private final String name;

  private int Timer;

  private final JSONArray json;

  /**
   * Constructs a new LedCustomAnimations instance.
   *
   * @param led LED driver.
   * @param ledBuffer LED buffer.
   * @param AnimationPath The name of the animation, usually the name of the json file minus
   *     ".json".
   * @param startTime Delay before starting the LED animation.
   * @param isLoop Whether to loop the animation or not.
   */
  public LedCustomAnimations(
      AddressableLED led,
      AddressableLEDBuffer ledBuffer,
      String AnimationPath,
      int startTime,
      boolean isLoop) {
    this.name = AnimationPath;
    this.led = led;
    this.ledBuffer = ledBuffer;
    this.isLoop = isLoop;
    this.isActive = false;
    this.Timer = -startTime;
    json = loadPath(AnimationPath);
  }

  public String getName() {
    return name;
  }

  public void setIsActive(boolean value) {
    isActive = value;
  }

  public boolean isActive() {
    return isActive;
  }

  public int getAnimationLength() {
    return json.toArray().length;
  }

  public void reset() {
    this.Timer = 0;
  }

  public void setLoop(boolean value) {
    this.isLoop = value;
  }

  public void setAnimation() {
    if (Timer < 0) {
      Timer++;
      return;
    }
    if (Timer >= getAnimationLength() && isLoop) Timer = 0;
    if (Timer >= getAnimationLength() && !isLoop) return;

    JSONObject frame = (JSONObject) json.get(Timer);
    long red = (long) frame.get("r");
    long green = (long) frame.get("g");
    long blue = (long) frame.get("b");

    long length = (long) frame.get("length");
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 0);
    }

    for (int i = 0; i < Math.floor(ledBuffer.getLength() * length / 100); i++) {
      ledBuffer.setRGB(i, (int) red, (int) green, (int) blue);
    }
    led.setData(ledBuffer);
    led.start();
    Timer++;
  }

  public JSONArray loadPath(String name) {
    JSONParser jsonParser = new JSONParser();
    // try (FileReader file = new FileReader(Filesystem.getDeployDirectory() +  "5829LedAnimations/"
    // + name + ".json")) {
    try (FileReader file =
        new FileReader(
            new File(Filesystem.getDeployDirectory(), "5829LedAnimations/" + name + ".json"))) {
      Object obj = jsonParser.parse(file);
      JSONArray json = (JSONArray) obj;
      return json;
    } catch (Exception e) {
      e.printStackTrace();
      return new JSONArray();
    }
  }

  public void end() {
    setLoop(false);
    Timer = getAnimationLength();
  }

  public boolean isFinished() {
    return Timer >= getAnimationLength();
  }
}
