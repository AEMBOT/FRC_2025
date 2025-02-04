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

        //elevator FF values because built in class doesn't work
    private double FeedForwardKs = 0.35;
    private double FeedForwardKg = 0.35;
    private double FeedForwardKv = 1.79;
    private double FeedForwardKa = 0.3;


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

        //TODO set proper tolerance value for our PID controller
        pivotPIDController.setTolerance(2, 0.1);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotAbsolutePosition = getAbsoluteEncoderPosition();
        inputs.pivotAppliedVolts = leadingMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotVelocity = leadingMotor.getVelocity().getValueAsDouble();
        inputs.pivotCurrentAmps = new double[] {leadingMotor.getStatorCurrent().getValueAsDouble(), 
                                                followingMotor.getStatorCurrent().getValueAsDouble()};
        inputs.pivotGoalPosition = Units.radiansToDegrees(pivotPIDController.getGoal().position);
        inputs.pivotSetpointPosition = Units.radiansToDegrees(pivotPIDController.getSetpoint().position);
        inputs.pivotSetpointVelocity = Units.radiansToRotations(pivotPIDController.getSetpoint().velocity);

        inputs.pivotAtGoal = pivotPIDController.atGoal();

        inputs.pivotOpenLoopStatus = pivotOpenLoop;
    }   

    /**
     * Calculates our pivot feedforward values.
     * @param positionDeg Position of our pivot in degrees.
     * @param velocityDegPerSec Velocity of our pivot in degrees per second.
     * @param accelDegPerSecSquared Acceleration of our pivot in degrees per second squared.
     * @return Our pivot feedforward values.
     */
    private double calculatePivotFeedforward(
        double positionDeg, double velocityDegPerSec, double accelDegPerSecSquared) {
      return FeedForwardKs * Math.signum(Units.degreesToRadians(velocityDegPerSec))
          + FeedForwardKg * Math.cos(Units.degreesToRadians(positionDeg))
          + FeedForwardKv * Units.degreesToRadians(velocityDegPerSec)
          + FeedForwardKa * Units.degreesToRadians(accelDegPerSecSquared);
    }

    /**
     * Gets the absolute encoder position of the pivot in degrees.
     * @return Returns the absolute encoder position of the pivot in degrees.
     */
    private double getAbsoluteEncoderPosition() {
        return (pivotEncoder.get() * 360) - pivotEncoderPositionOffset;
    }

    /**
     * Gets the pivot's velocity in degrees per second.
     * @return Returns the pivot's velocity in degrees per second.
     */
    private double getPivotVelocity() {
        return (leadingMotor.getVelocity().getValueAsDouble() * 360);
    }

    /**
     * Sets the motor voltage of our pivot and clamps it between our min and max angle.
     * @param volts Amount of volts sent to our pivot motors.
     */
    private void setMotorVoltage(double volts) {
        if (getAbsoluteEncoderPosition() < pivotMinAngle) {
            volts = clamp(volts, -Double.MAX_VALUE, 0);
        }
        if (getAbsoluteEncoderPosition() > pivotMaxAngle) {
            volts = clamp(volts, 0, Double.MAX_VALUE);
        }
        leadingMotor.setVoltage(volts);
    }

    // TODO Find values for the interpolation of our feedforward and feedback values
    /**
     * Interpolates our pivot feedforward and PID constants.
     * @param elevatorPositionMet Current position of the elevator in meters.
     */
    private void interpolateConstants(double elevatorPositionMet) {
        pivotPIDController.setPID(
            0, 
            0, 
            0
            );
        FeedForwardKs = 0.0 * elevatorPositionMet;
        FeedForwardKv = 0.0 * elevatorPositionMet;
        FeedForwardKg = 0.0 * elevatorPositionMet;
        FeedForwardKa = 0.0 * elevatorPositionMet;
    }

    @Override
    public void setAngle(double goalAngleDeg, double elevatorPositionMet) {
        pivotOpenLoop = false;
        interpolateConstants(elevatorPositionMet);
        goalAngleDeg = clamp(goalAngleDeg, pivotMinAngle, pivotMaxAngle);

        double pidOutput = pivotPIDController.calculate(
            getAbsoluteEncoderPosition(), 
            goalAngleDeg);
        double acceleration = (pivotPIDController.getSetpoint().velocity - lastVelocity) / (Timer.getFPGATimestamp() - lastTime);
        double feedForward = calculatePivotFeedforward(
            pivotPIDController.getSetpoint().position,
            pivotPIDController.getSetpoint().velocity, 
            acceleration);
        
        setVoltage(
            pidOutput
            + feedForward);
        lastVelocity = pivotPIDController.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();   

        Logger.recordOutput("Pivot/CalculatedFFVolts", feedForward);
        Logger.recordOutput("Pivot/PIDCommandVolts", pidOutput);

        Logger.recordOutput("Pivot/VelocityError", pivotPIDController.getVelocityError());
        Logger.recordOutput("Pivot/PositionError", pivotPIDController.getPositionError());
        
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
        pivotPIDController.reset(getAbsoluteEncoderPosition(), getPivotVelocity());
    }
}