package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristConstants {
  // FIXME Put soft stops back when zeroing works
  /** Maximum angle for the wrist to move to, in degrees */
  public static final double MAX_ANGLE = 160;

  /** Minimum angle for the wrist to move to, in degrees */
  public static final double MIN_ANGLE = -18;

  /** */
  public static final float VOLTAGE_LIMIT = 5;

  /** ID of the wrist sparkmax */
  public static final int MOTOR_ID = 14;

  /** */
  public static final int MOTOR_CURRENT_LIMIT = 120;

  /** */
  public static final int ENCODER_ID = 2;

  /** */
  public static final double MOTOR_RATIO = 25;

  /** */
  public static final double ENCODER_POSITION_OFFSET = 105.3502 * -1;

  /** */
  public static final double GEAR_RATIO = 6;

  /** How many degrees the wrist can be off its goal position for it to be sufficient */
  public static final double ALLOWED_DEVIANCE = 6;

  /** */
  public static final double ZERO_POSITION = 29.734375 * -1; // TODO tune

  /** */
  public static final double ZEROING_VOLTAGE = 1; // TODO tune

  /** */
  public static final double WRIST_ZEROING_MAX_AMPS = 20; // TODO tune

  /** */
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
