package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

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

  // Procedure: Set offset to 0, move pivot to real 90 (vertical). Set encoder offset to 90 - output
  // value.
  public static final double ENCODER_POSITION_OFFSET = 90 - 247.87962;

  /** */
  public static final double GEAR_RATIO = 378;

  /** */
  public static final ArmFeedforward FF_MODEL =
      new ArmFeedforward(0.11164, 0.0090459, 0.11954, 0.0090459);

  /** */
  public static final PIDController PID_CONTROLLER = new PIDController(.09361, 0, 0);

  /** */
  public static final TrapezoidProfile TRAPEZOID_PROFILE =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 2));

  /** Ramp Rate of the pivot System ID in volts per second */
  public static final double SYS_ID_RAMP_RATE = 1;

  /** Setp Voltage of the pivot System ID in volts */
  public static final double SYS_ID_STEP_VALUE = 7;

  /** Timeout of the pivot System ID in volts */
  public static final double SYS_ID_TIMEOUT = 30;

  /** How many degrees the pivot can be off its goal position for it to be sufficient */
  public static final double ALLOWED_DEVIANCE = 1.15;

  /** */
  public static final Translation3d TRANSLATION_FROM_ROBOT = new Translation3d(-0.2, 0, 0.255);

  /** */
  public static final double DEFAULT_ANGLE = 90;

  /** */
  public static final double SIM_GOAL_POSITION = 1.05;

  /** */
  public static final double SIM_SETPOINT_POSITION = 1.05;

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

  public static final int RATCHET_PIN_1_ID = 8;
  public static final int RATCHET_PIN_2_ID = 9;
}
