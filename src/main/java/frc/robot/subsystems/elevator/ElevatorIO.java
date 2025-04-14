package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    /** Current position of the elevator in meters(this isn't absolute, right?) */
    public double elevatorAbsolutePosition = 0.0;

    /** Current velocity of the elevator in meters per second */
    public double elevatorAbsoluteVelocity = 0.0;

    /** Current applied voltage to the elevator */
    public double elevatorAppliedVolts = 0.0;

    /**
     * Currents amps applied to each motor. Both motors are logged indivually to better find issues.
     */
    public double[] elevatorCurrentAmps = new double[] {0, 0};

    /** Goal position of the elevator in meters */
    public double elevatorGoalPosition = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Sets the target height of our elevator, in meters */
  public default void setHeight(double height) {}

  /** Directly sets the voltage without any safety clamps. */
  public default void setVoltage(double volts) {}

  /** Limit our height based on the angle that we feed into it from pivot */
  public default void limitHeight(double pivotAngle) {}

  /**
   * Resets our elevator goal to our current position. Really useful when we manually change our
   * target and then tell our elevator to stop, but our "goal" position has not been reached.
   */
  public default void resetProfile() {}

  /** Sets the offset of the elevator to a known position */
  public default void setMotorZero() {}
}
