package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotConstants {
  /** Maximum angle for the pivot to move to, in degrees */
  public static final double MAX_ANGLE = 120;

  /** Minimum angle for the pivot to move to, in degrees */
  public static final double MIN_ANGLE = 0.0;

  /** ID of the left pivot sparkmax */
  public static final int LEFT_MOTOR_ID = 10;

  /** */
  public static final boolean LEFT_MOTOR_INVERTED = true;

  /** */
  public static final int LEFT_MOTOR_CURRENT_LIMIT = 80;

  /** ID of the right pivot sparkmax */
  public static final int RIGHT_MOTOR_ID = 11;

  /** */
  public static final boolean RIGHT_MOTOR_INVERTED = false;

  /** */
  public static final int RIGHT_MOTOR_CURRENT_LIMIT = 80;

  /** */
  public static final int ENCODER_ID = 1;

  // Procedure: Set offset to 0, move pivot to real 90 (vertical). Set encoder offset to 90 - output
  // value.
  public static final double ENCODER_POSITION_OFFSET = 90 - 247.87962;

  /** */
  public static final double GEAR_RATIO = 378;

  /** How many degrees the pivot can be off its goal position for it to be sufficient */
  public static final double ALLOWED_DEVIANCE = 1.15;

  public static final int RATCHET_PIN_1_ID = 8;
  public static final int RATCHET_PIN_2_ID = 9;

  public static final SingleJointedArmSim SIM =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2),
          378,
          SingleJointedArmSim.estimateMOI(2, 1),
          1,
          Units.degreesToRadians(MIN_ANGLE),
          Units.degreesToRadians(MAX_ANGLE),
          true,
          0);
}
