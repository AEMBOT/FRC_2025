package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  
    @AutoLog
    public static class IntakeIOInputs {
        public double intakeAppliedVolts = 0.0;
    }
  
    public default void updateInputs(IntakeIOInputs inputs) {}
  
    public default void setVoltage(double volts) {}


}


