package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    /** Current angle of the elevator in inches */
    public double elevatorAbsolutePosition = 0.0;

    /** Current velocity the elevator in travelling at in rotations per minute */
    public double elevatorAbsoluteVelocity = 0.0;

    /** */
    public double elevatorAppliedVolts = 0.0;

    /**
     * Currents amps applied to each motor. Both motors are logged indivually to better find issues.
     */
    public double[] elevatorCurrentAmps = new double[] {0, 0};

    /** Goal position of the elevator in inches */
    public double elevatorGoalPosition = 45;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Sets the angle of elevator, in meters */
  public default void setHeight(double height) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void limitHeight(double pivotAngle) {}

  /** Resets the elevator goal */
  public default void resetProfile() {}

  /** Sets the offset of the elevator to a known position */
  public default void setMotorZero() {}

  public default void simulationPeriodic() {}
}
