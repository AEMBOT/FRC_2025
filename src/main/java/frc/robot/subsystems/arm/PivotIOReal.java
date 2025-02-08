package frc.robot.subsystems.arm;

import static frc.robot.Constants.PivotConstants.*;
import static edu.wpi.first.math.MathUtil.clamp;

import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class PivotIOReal implements PivotIO {
    
    private boolean pivotOpenLoop = true;
    private final TalonFX leadingMotor = new TalonFX(pivotLeftMotorID);
    private final TalonFX followingMotor = new TalonFX(pivotRightMotorID);

    private double lastTime;
    private double lastVelocity;

    private double FeedForwardKs = pivotFFValues[0];
    private double FeedForwardKg = pivotFFValues[1];
    private double FeedForwardKv = pivotFFValues[2];
    private double FeedForwardKa = pivotFFValues[3];


    public PivotIOReal() {

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

        leftMotorConfig.CurrentLimits.StatorCurrentLimit = pivotLeftMotorCurrentLimit;
        leftMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        rightMotorConfig.CurrentLimits.StatorCurrentLimit = pivotRightMotorCurrentLimit;
        rightMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leadingMotor.setNeutralMode(NeutralModeValue.Brake);
        followingMotor.setNeutralMode(NeutralModeValue.Brake);

        followingMotor.setControl(new Follower(pivotLeftMotorID, true));

        pivotPIDController.setTolerance(pivotAngleToleranceDeg, pivotVelocityToleranceDegPerSec);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotAbsolutePosition = getAbsoluteEncoderPosition();
        inputs.pivotAppliedVolts = leadingMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotVelocity = getPivotVelocity();
        inputs.pivotCurrentAmps = new double[] {leadingMotor.getStatorCurrent().getValueAsDouble(), 
                                                followingMotor.getStatorCurrent().getValueAsDouble()};
        inputs.pivotGoalPosition = Units.radiansToDegrees(pivotPIDController.getGoal().position);
        inputs.pivotSetpointPosition = Units.radiansToDegrees(pivotPIDController.getSetpoint().position);
        inputs.pivotSetpointVelocity = Units.radiansToDegrees(pivotPIDController.getSetpoint().velocity);

        inputs.pivotAtGoal = pivotPIDController.atGoal();
        inputs.pivotAtSetpoint = pivotPIDController.atSetpoint();

        inputs.pivotOpenLoopStatus = pivotOpenLoop;
    }   

    /**
     * Calculates our pivot feedforward values.
     * @param positionRad Setpoint position of our pivot in radians.
     * @param velocityRadPerSec Setpoint velocity of our pivot in radians per second.
     * @param accelRadPerSecSquared Setpoint acceleration of our pivot in radians per second squared.
     * @return Our pivot feedforward values.
     */
    private double calculatePivotFeedforward(
        double positionRad, double velocityRadPerSec, double accelRadPerSecSquared) {
      return FeedForwardKs * Math.signum(velocityRadPerSec)
          + FeedForwardKg * Math.cos(positionRad)
          + FeedForwardKv * (velocityRadPerSec)
          + FeedForwardKa * (accelRadPerSecSquared);
    }

    /**
     * Gets the absolute encoder position of the pivot in degrees.
     * @return Returns the absolute encoder position of the pivot in degrees.
     */
    private double getAbsoluteEncoderPosition() {
        return Units.rotationsToDegrees(pivotEncoder.get() - pivotEncoderPositionOffset);
    }

    /**
     * Gets the pivot's velocity in degrees per second.
     * @return Returns the pivot's velocity in degrees per second.
     */
    private double getPivotVelocity() {
        return Units.rotationsToDegrees(leadingMotor.getVelocity().getValueAsDouble());
    }

    /**
     * Sets the motor voltage of our pivot and clamps it between our min and max angle.
     * @param volts Amount of volts sent to our pivot motors.
     */
    private void setMotorVoltage(double volts) {
        if (getAbsoluteEncoderPosition() < pivotMinAngle) {
            volts = clamp(volts, -Double.MAX_VALUE, 0);
        } //volts need to be characterized to be enough to beat static friction right?
        if (getAbsoluteEncoderPosition() > pivotMaxAngle) {
            volts = clamp(volts, 0, Double.MAX_VALUE);
        }
        leadingMotor.setVoltage(volts);
    }

    /**
     * Interpolates our pivot feedforward and PID constants.
     * @param elevatorPositionMet Current position of the elevator in meters.
     */
    private void interpolateConstants(double elevatorPositionMet) {
        //pivotPIDController.setPID(
        //    0 * PivotPIDFactor, 
        //    0 * PivotPIDFactor, 
        //    0 * PivotPIDFactor);
        //FeedForwardKs = 0.0 * elevatorPositionMet * PivotFFactor;
        //FeedForwardKv = 0.0 * elevatorPositionMet * PivotFFactor;
        //FeedForwardKg = 0.0 * elevatorPositionMet * PivotFFactor;
        //FeedForwardKa = 0.0 * elevatorPositionMet * PivotFFactor;
    }

    @Override
    public void setAngle(double goalAngleDeg, double elevatorPositionMet) {
        pivotOpenLoop = false;

        interpolateConstants(elevatorPositionMet);

        goalAngleDeg = clamp(goalAngleDeg, pivotMinAngle, pivotMaxAngle);

        double pidOutput = pivotPIDController.calculate( //convert to radians for our FF equation
            Units.degreesToRadians(getAbsoluteEncoderPosition()), 
            Units.degreesToRadians(goalAngleDeg));

        double acceleration = ( //acceleration in radians
            pivotPIDController.getSetpoint().velocity - lastVelocity) 
            / (Timer.getFPGATimestamp() - lastTime);

        double feedForward = calculatePivotFeedforward( //feedforward in radians
            pivotPIDController.getSetpoint().position,
            pivotPIDController.getSetpoint().velocity, 
            acceleration);

        Logger.recordOutput("Pivot/CalculatedFFVolts", feedForward);
        Logger.recordOutput("Pivot/PIDFeedbackVolts", pidOutput);
        //convert back to degrees
        Logger.recordOutput("Pivot/VelocityErrorDeg", Units.radiansToDegrees(pivotPIDController.getVelocityError()));
        Logger.recordOutput("Pivot/PositionErrorDeg", Units.radiansToDegrees(pivotPIDController.getPositionError()));
        
        setMotorVoltage(feedForward + pidOutput);

        lastVelocity = pivotPIDController.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void setVoltage(double volts) {
        pivotOpenLoop = true;
        setMotorVoltage(volts);
    }


    @Override
    public void pivotResetProfile() {
        pivotPIDController.reset(Units.degreesToRadians(getAbsoluteEncoderPosition()), 0);
    }
}