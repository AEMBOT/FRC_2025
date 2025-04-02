package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristConstants {
  // FIXME Put soft stops back when zeroing works
  /** Maximum angle for the wrist to move to, in degrees */
  public static final double MAX_ANGLE = 360;

  /** Minimum angle for the wrist to move to, in degrees */
  public static final double MIN_ANGLE = -360;

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
  public static final PIDController PID_CONTROLLER = new PIDController(0, 0, 0);

  /** */
  public static final TrapezoidProfile TRAPEZOID_PROFILE =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 2));

  /** Ramp Rate of the wrist System ID in volts per second */
  public static final double SYS_ID_RAMP_RATE = 1;

  /** Setp Voltage of the wrist System ID in volts */
  public static final double SYS_ID_STEP_VALUE = 7;

  /** Timeout of the wrist System ID in volts */
  public static final double SYS_ID_TIMEOUT = 30;

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
  public static final double ZERO_POSITION = -41.11875; // TODO tune

  /** */
  public static final double ZEROING_VOLTAGE = 1; // TODO tune

  /** */
  public static final double WRIST_ZEROING_MAX_AMPS = 10; // TODO tune

  /** */
  public static final SingleJointedArmSim SIM =
      new SingleJointedArmSim(
          DCMotor.getNEO(2),
          300,
          0.17,
          0.500,
          Units.degreesToRadians(MIN_ANGLE),
          Units.degreesToRadians(MAX_ANGLE),
          true,
          Units.degreesToRadians(45));
}
