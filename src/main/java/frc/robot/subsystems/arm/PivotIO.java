package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.util.Units;

public interface PivotIO {
    
    @AutoLog
    public static class PivotIOInputs {
        public boolean pivotAtGoal = true;
        /** Current angle of the pivot in degrees */
        public double pivotAbsolutePosition = 0.0;
        /** Current velocity the pivot in travelling at in rotations per minute */
        public double pivotAbsoluteVelocity = 0.0;
        /**  */
        public double pivotVelocity = 0.0;

        public double pivotAppliedVolts = 0.0;
        /** Currents amps applied to each motor. Both motors are logged indivually to better find issues. */
        public double[] pivotCurrentAmps = new double[] {};
        /** Goal position of the pivot in degrees */
        public double pivotGoalPosition = Units.degreesToRadians(50);
        /** Setpoint position of the pivot in degrees */
        public double pivotSetpointPosition = 0.0;
        /** Setpoint position of the pivot in rotations per second TODO confirm that this is in rpm and not radpersec */
        public double pivotSetpointVelocity = 0.0;
        /** Whether the pivot subsystem is running in an openloop */
        public boolean pivotOpenLoopStatus = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(PivotIOInputs inputs) {}

    /** Sets the angle of pivot, in degrees */
    public default void setAngle(double angle, double elevatorPositionMet) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {}

    /** Resets the pivot goal and setpoint to the current angle of the pivot*/
    public default void pivotResetProfile () {}
}