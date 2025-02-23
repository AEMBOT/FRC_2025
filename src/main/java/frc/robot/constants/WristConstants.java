package frc.robot.constants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class WristConstants {
  public static final double encoderOffset = (194.10106985252673 - 84 + 123.1) * -1;
  public static final double wristMaxAngle = -90;
  public static final double wristMinAngle = 90;
  public static final double deadzone = 5.0;
  public static final double wristAngleToleranceDeg = 1.0;
  public static final double wristVelocityTolerangeDegPerSec = 0.5;

  /* Device IDs */
  public static final int motorID = 14;
  public static final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(2);

  public static final TrapezoidProfile.Constraints wristConstraints =
      new TrapezoidProfile.Constraints(1, 3.5);

  public static final ProfiledPIDController wristPIDController =
      switch (GeneralConstants.currentRobot) {
        case DORY -> new ProfiledPIDController(0.075499, 0, 0.00, wristConstraints);
        case NAUTILUS -> new ProfiledPIDController(0.075499, 0, 0.00, wristConstraints);
      };

  public static final double[] wristFFValues = // in deg
      switch (GeneralConstants.currentRobot) {
        case DORY -> new double[] {0.33149, 0.077915, 0.0042737, 0.0};
        case NAUTILUS -> new double[] {0.2, 0.05, 0.004, 0.01}; // ks, kg, kv, ka
      };

  public static final double WristPIDFactor =
      switch (GeneralConstants.currentRobot) {
        case DORY -> 0.0;
        case NAUTILUS -> 0.0;
      };
  public static final double wristMotorCurrentLimit = 0.25;

  public static final double WristFFactor =
      switch (GeneralConstants.currentRobot) {
        case DORY -> 0.0;
        case NAUTILUS -> 0.0;
      };
}
