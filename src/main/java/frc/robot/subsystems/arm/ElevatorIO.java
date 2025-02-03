package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean elevatorAtGoal = true;
        public boolean elevatorOpenLoopStatus = false;
        public double elevatorMotorRotation = 0.0;
        public double elevatorVoltage = 0.0;
        public double elevatorVelocity = 0.0;
        public double elevatorPosition = 0.0;
        public double elevatorGoalPosition = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setGoalPosition(double goalPosition, double pivotAngleDeg) {}

    /** Sets the voltage of the elevator motor. */
    public default void setVoltage(double voltage) {}

    public default void elevatorResetProfile() {}
}