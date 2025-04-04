package frc.robot.constants;

public class PivotConstants {
  /** Maximum angle for the pivot to move to, in degrees */
  public static final double MAX_ANGLE = 120;

  /** Minimum angle for the pivot to move to, in degrees */
  public static final double MIN_ANGLE = 3;

  /** */
  public static final float VOLTAGE_LIMIT = 5;

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

  // Procedure: Set offset to -90, move pivot to real 90 (Vertical). Output value with flipped sign
  // is new offset.
  public static final double ENCODER_POSITION_OFFSET = 90 - 248.22256220556403;

  /** */
  public static final double GEAR_RATIO = 378;

  /** How many degrees the pivot can be off its goal position for it to be sufficient */
  public static final double ALLOWED_DEVIANCE = 1.15;

  public static final int RATCHET_PIN_1_ID = 8;
  public static final int RATCHET_PIN_2_ID = 9;
}
