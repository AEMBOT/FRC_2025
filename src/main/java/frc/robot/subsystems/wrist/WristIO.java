package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

  @AutoLog
  public static class WristIOInputs {
    /** Current angle of the wrist in degrees */
    public double wristAbsolutePosition = 0.0;

    /** Current velocity the wrist in travelling at in rotations per minute */
    public double wristAbsoluteVelocity = 0.0;

    /** */
    public double wristAppliedVolts = 0.0;

    /**
     * Currents amps applied to each motor. Both motors are logged indivually to better find issues.
     */
    public double[] wristCurrentAmps = new double[] {};

    /** Goal position of the wrist in degrees */
    public double wristGoalPosition = 45;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /** Sets the angle of wrist, in degrees */
  public default void setAngle(double angle) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Resets the wrist goal during manual control */
  public default void resetGoal() {}
}
