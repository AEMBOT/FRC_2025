package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
      public double pivotAbsolutePositionRad = 0.0;
      public double pivotAbsoluteVelocityRadPerSec = 0.0;
      public double pivotAppliedVolts = 0.0;
      public double[] pivotCurrentAmps =
          new double[] {}; // Log motors individually, useful for failure analysis
  
      public double pivotGoalPosition = 0.0;
    }
  
    /** Updates the set of loggable inputs. */
    public default void updateInputs(PivotIOInputs inputs) {}
  
    /** Sets the angle of the pivot, in degrees. */
    public default void setPosition(double positionDeg) {}
  }
