package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
      public double pivotAbsolutePositionDeg = 0.0;
      public double pivotErrorDeg = 0.0;
      public double pivotVelocityDegPerSec = 0.0;
      public double pivotAppliedVolts = 0.0;
      public double[] pivotCurrentSupplyAmps =
          new double[] {}; // Log motors individually, useful for failure analysis
      public double[] pivotCurrentStatorAmps =
          new double[] {}; // Log motors individually, useful for failure analysis
  
      public double pivotSetpoint = 0.0;
    }
  
    /** Updates the set of loggable inputs. */
    public default void updateInputs(PivotIOInputs inputs) {}
  
    /** Sets the angle of the pivot, in degrees. */
    public default void setPosition(double positionDeg) {}

    public default void periodic() {}
  }
