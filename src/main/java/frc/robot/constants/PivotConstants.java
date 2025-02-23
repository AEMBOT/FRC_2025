package frc.robot.constants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public final class PivotConstants {
  /** Maximum angle for the pivot to move to, in degrees */
  public static final double pivotMaxAngle = 100;

  /** Minimum angle for the pivot to move to, in degrees */
  public static final double pivotMinAngle = 30;

  /** ID of the left pivot sparkmax */
  public static final int pivotLeftMotorID = 10;

  /** */
  public static final boolean pivotLeftMotorInverted = false;

  /** */
  public static final int pivotLeftMotorCurrentLimit = 50;

  /** ID of the right pivot sparkmax */
  public static final int pivotRightMotorID = 11;

  /** */
  public static final boolean pivotRightMotorInverted = false;

  /** */
  public static final int pivotRightMotorCurrentLimit = 50;

  /** */
  public static final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(1);

  /** */
  public static final double pivotEncoderPositionOffset = -156.67488616687214;

  /** */
  public static final double gearRatio = 93.3333333;

  /** */
  public static final TrapezoidProfile.Constraints pivotConstraints =
      new TrapezoidProfile.Constraints(Units.radiansToDegrees(2), Units.radiansToDegrees(5));

  public static final ProfiledPIDController pivotPIDController =
      switch (GeneralConstants.currentRobot) {
        case DORY -> new ProfiledPIDController(10, 0, 0, pivotConstraints);
        case NAUTILUS -> new ProfiledPIDController(0.0, 0.0, 0.0, pivotConstraints);
      };

  public static final double[] pivotFFValues = // in radians
      switch (GeneralConstants.currentRobot) {
        case DORY -> new double[] {0.0, 0.0, 0.0, 0.0};
        case NAUTILUS -> new double[] {0.0, 0.16, 4.15, 0.01}; // ks kg kv ka
      };

  public static final double PivotPIDFactor =
      switch (GeneralConstants.currentRobot) {
        case DORY -> 0.0;
        case NAUTILUS -> 0.0;
      };

  public static final double PivotFFactor =
      switch (GeneralConstants.currentRobot) {
        case DORY -> 0.0;
        case NAUTILUS -> 0.0;
      };

  /** How many degrees the pivot can be off its goal position for it to be sufficient */
  public static final double pivotAngleToleranceDeg = 1.15;

  /** */
  public static final double pivotVelocityToleranceDegPerSec = 1.15;

  public static final Translation3d pivotTranslationFromRobot = new Translation3d(-0.2, 0, 0.255);

  /** */
  public static final double pivotDefaultAngle = 90;

  /** */
  public static final double pivotSimGoalPosition = 1.05;

  /** */
  public static final double pivotSimSetpointPosition = 1.05;

  /** */
  public static final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(2),
          300,
          0.17,
          0.500,
          Units.degreesToRadians(pivotMinAngle),
          Units.degreesToRadians(pivotMaxAngle),
          true,
          Units.degreesToRadians(45));
}
