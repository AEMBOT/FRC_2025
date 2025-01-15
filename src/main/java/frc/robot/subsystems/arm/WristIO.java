package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public double position = 0.0;
        public double setpoint = 0.0;
        public double error = 0.0;

        public double appliedVoltage = 0.0;
    }

    public default void periodic() {}

    /** Updates the set of loggable inputs. */
    public default void updateInputs(WristIOInputs inputs) {}

    /**
     * Sets the wrist rotation setpoint.
     * @param position The rotation setpoint in degrees, clamped between
     */
    public default void setSetpoint(Double setpoint) {}
}
