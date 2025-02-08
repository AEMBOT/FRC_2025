package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double elevatorMotorRotation = 0.0;
        public double elevatorVoltage = 0.0;
        public double elevatorCurrentDraw = 0.0;
        public double elevatorMaxPos = 0.0;
        public double elevatorMinPos = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Sets the voltage of the elevator motor. */
    public default void setVoltage(double voltage) {}
}