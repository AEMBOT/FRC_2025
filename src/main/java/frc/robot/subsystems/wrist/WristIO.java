package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

  @AutoLog
  public static class WristIOInputs {
    /** Current angle of the wrist in degrees */
    public double wristAbsolutePosition = 0.0;

    /** Current angle measured by wrist abs encoder, in rotations */
    public double wristAbsoluteEncoderRawRotations = 0.0;

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

    /** Setpoint position of the wrist in degrees */
    public double wristSetpointPosition = 45;

    /**
     * Setpoint position of the wrist in rotations per second TODO confirm that this is in rpm and
     * not radpersec
     */
    public double wristSetpointVelocity = 0.0;

    /** Whether the wrist subsystem is running in an openloop */
    public boolean openLoopStatus = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /** Sets the angle of wrist, in degrees */
  public default void setAngle(double angle) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Resets the wrist goal and setpoint to the current angle of the wrist */
  public default void resetProfile() {}
}
