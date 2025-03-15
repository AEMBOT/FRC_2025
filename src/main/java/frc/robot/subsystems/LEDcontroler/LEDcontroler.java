package frc.robot.subsystems.LEDcontroler;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import java.util.Optional;

public class LEDcontroler {
  private final SerialPort LEDs; // init the LEDs variable


  public LEDcontroler() {
    LEDs = new SerialPort(115200, SerialPort.Port.kMXP);
  }

  public void getalliance(){
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.get() == Alliance.Red) { // found red Alliance and sets the Alliance color to red
      LEDDO("r");
    }
    if (alliance.get() == Alliance.Blue) { // found blue Alliance and sets the Alliance color to blue
      LEDDO("b");
    }
  }

  public void LEDDO(String what) {
    LEDs.writeString(what); // sends a character through Serial
  }
}
