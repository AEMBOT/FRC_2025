package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    
    @AutoLog
    public static class PivotIOInputs {
        /** Whether the pivot is at the goal position or not */
        public boolean pivotAtGoal = true;

        public boolean pivotAtSetpoint = true;

        /** Current angle of the pivot in degrees */
        public double pivotAbsolutePosition = 0.0;
    /** Current velocity the pivot in travelling at in rotations per minute */
    public double pivotAbsoluteVelocity = 0.0;
        /** Current volts applied to the pivot.*/
        public double pivotAppliedVolts = 0.0;
        /** Currents amps applied to each motor. Both motors are logged indivually to better find issues. */
        public double[] pivotCurrentAmps = new double[] {};
        /** Goal position of the pivot in degrees */
        public double pivotGoalPosition = 0.0;
        /** Setpoint position of the pivot in degrees */
        public double pivotSetpointPosition = 0.0;
        /** Setpoint position of the pivot in degrees per second */
        public double pivotSetpointVelocity = 0.0;
        /** Whether the pivot subsystem is running in an openloop */
        public boolean pivotOpenLoopStatus = false;

        public double rawEncoderValue = 0.0;

        public double TheotreticalVoltage = 0.0;
      }
    }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

    /** Sets the angle of pivot, in degrees */
    public default void setAngle(double goalAngleDeg, double elevatorPositionMet) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

    /** Resets the pivot goal and setpoint to the current angle of the pivot*/
    public default void pivotResetProfile () {}
}
