package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristConstants {
  /** Maximum angle for the wrist to move to, in degrees */
  public static final double MAX_ANGLE = 135;

  /** Minimum angle for the wrist to move to, in degrees */
  public static final double MIN_ANGLE = -40;

  /** */
  public static final float VOLTAGE_LIMIT = 5;

  /** ID of the wrist sparkmax */
  public static final int MOTOR_ID = 14;

  /** */
  public static final boolean MOTOR_INVERTED = false;

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

  /** */
  public static final ArmFeedforward FF_MODEL = new ArmFeedforward(0, 0, 0, 0);

  /** */
  public static final PIDController PID_CONTROLLER = new PIDController(5, 0, 0);

  /** How many degrees the wrist can be off its goal position for it to be sufficient */
  public static final double ALLOWED_DEVIANCE = 6;

  /** */
  public static final Translation3d TRANSLATION_FROM_ROBOT = new Translation3d(-0.2, 0, 0.255);

  /** */
  public static final double DEFAULT_ANGLE = 0;

  /** */
  public static final double SIM_GOAL_POSITION = 1.05;

  /** */
  public static final double SIM_SETPOINT_POSITION = 1.05;

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
