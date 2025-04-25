package frc.robot.constants;

import edu.wpi.first.wpilibj.DigitalInput;

public class GeneralConstants {
  /**
   * A jumper that identifies what robot we're currently running on. It outputs high voltage on
   * Nautilus and low voltage on Dory. If there is no jumper, it'll default to high/true
   */
  public static final DigitalInput robotJumper = new DigitalInput(0);

  public static final Robot currentRobot;

  static {
    if (robotJumper.get()) {
      currentRobot = Robot.NAUTILUS;
    } else {
      currentRobot = Robot.DORY;
    }
  }

  public static final Mode currentMode = Mode.REPLAY;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY,
  }

  public enum Robot {
    NAUTILUS,
    DORY
  }

  public static final double UPDATE_PERIOD = 0.02;
}
