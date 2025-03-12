package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimConstants {

  public static final double PIVOT_MIN_ANGLE = 3;
  public static final double PIVOT_MAX_ANGLE = 120;

  public static final double WRIST_MAX_ANGLE = 135;
  public static final double WRIST_MIN_ANGLE = -15;

  public static final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2),
          378,
          SingleJointedArmSim.estimateMOI(1, 20),
          1,
          Units.degreesToRadians(PIVOT_MIN_ANGLE),
          Units.degreesToRadians(PIVOT_MAX_ANGLE),
          true,
          0,
          0.001,
          0.0);

  public static final SingleJointedArmSim wristSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2),
          6,
          SingleJointedArmSim.estimateMOI(0.2, 5),
          1,
          Units.degreesToRadians(WRIST_MIN_ANGLE),
          Units.degreesToRadians(WRIST_MAX_ANGLE),
          true,
          0,
          0.0001,
          0.0);
}
