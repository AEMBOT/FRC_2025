package frc.robot.subsystems.LEDcontroller ;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import java.util.Optional;

public class LedController  {
  private final SerialPort LEDs; // init the LEDs variable

  public LedController () {
    LEDs = new SerialPort(115200, SerialPort.Port.kMXP); // init the LEDs Serial Port
  }

  public void getalliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.get() == Alliance.Red) { // found red Alliance and sets the Alliance color to red
      LEDDO("r");
    }
    if (alliance.get()
        == Alliance.Blue) { // found blue Alliance and sets the Alliance color to blue
      LEDDO("b");
    }
  }
  /** sets led the color or patern of the leds
  * @param what the character so send to the arduino
  */
  public void LEDDO(String what) {
    LEDs.writeString(what); // sends a character through Serial
  }
}
