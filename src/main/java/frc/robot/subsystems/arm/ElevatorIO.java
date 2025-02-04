package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        /** Whether the elevator is at goal posiiton or not */
        public boolean elevatorAtGoal = true;
        /** Elevator motor rotation in degrees per second */
        public double elevatorMotorRotation = 0.0;
        /** Current elevator voltage */
        public double elevatorVoltage = 0.0;
        /** Current elevator velocity in rotations per second */
        public double elevatorVelocity = 0.0;
        /** Current elevator position in meters */
        public double elevatorPosition = 0.0;
        /** Current elevator goal position in meters */
        public double elevatorGoalPosition = 0.0;
        /** Whether the elevator is running open loop or not */
        public boolean elevatorOpenLoopStatus = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setGoalPosition(double goalPosition, double pivotAngleDeg) {}

    /** Sets the voltage of the elevator motor. */
    public default void setVoltage(double voltage) {}

    public default void elevatorResetProfile() {}
}