package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public boolean wristAtGoal = true;

        public double wristAbsAngle = 0.0;
        public double wristAbsVelocity = 0.0;

        public double wristGoal = 0.0;
        public double wristSetpoint = 0.0;

        public double wristAppliedVoltage = 0.0;
        public double wristCurrentAmps = 0.0;

        public boolean wristOpenLoopStatus = false;
    }

    public default void periodic() {}

    /** Updates the set of loggable inputs. */
    public default void updateInputs(WristIOInputs inputs) {}

    /**
     * Sets the wrist rotation setpoint.
     * @param position The rotation setpoint in degrees, clamped between
     */
    public default void setAngle(double angle, double elevatorPosMet, double pivotAngleDeg) {}

    public default void wristResetProfile() {}

    public default void setCharacterizationVoltage(double volts) {}
}
