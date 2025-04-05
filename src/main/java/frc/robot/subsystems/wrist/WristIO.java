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

    /** Currents amps applied to the motor */
    public double wristCurrentAmps = 0.0;

    /** Goal position of the wrist in degrees */
    public double wristGoalPosition = 45;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /** Sets the angle of wrist, in degrees */
  public default void setAngle(double angle) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void simulationPeriodic() {}

  /** Resets the wrist goal and setpoint to the current angle of the wrist */
  public default void resetProfile() {}

  /** Sets the offset of the wrist to a known position */
  public default void setMotorZero() {}
}
