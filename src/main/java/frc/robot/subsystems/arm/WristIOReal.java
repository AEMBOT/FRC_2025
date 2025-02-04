package frc.robot.subsystems.arm;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.Constants.UPDATE_PERIOD;
import static frc.robot.Constants.WristConstants.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class WristIOReal implements WristIO {

    private boolean OpenLoopStatus = true;
    private final TalonFX motor = new TalonFX(motorID);

    private double lastTime;
    private double lastVelocity;

    // Wrist FF variables
    private double FeedForwardKs = 0.35;
    private double FeedForwardKv = 0.35;
    private double FeedForwardKa = 1.79;

    public WristIOReal() {

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.StatorCurrentLimit = wristMotorCurrentLimit;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.setNeutralMode(NeutralModeValue.Brake);

        //TODO set proper tolerance value for our PID controller
        wristPIDController.setTolerance(2, 0.1);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.wristAbsAngle = getAbsoluteAngle();
        inputs.wristVelocity = getWristVelocity();
        inputs.wristGoal = Units.radiansToDegrees(wristPIDController.getGoal().position);
        inputs.wristSetpoint = Units.radiansToDegrees(wristPIDController.getSetpoint().position);
        inputs.wristAppliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.wristCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
        inputs.wristOpenLoopStatus = OpenLoopStatus;
        inputs.wristAtGoal = wristPIDController.atGoal();
    }

    /**
     * Sets the voltage of the wrist motor.
     * @param volts The voltage to set.
     */
    private void setMotorVoltage(double volts) {
        OpenLoopStatus = true;
        if (getAbsoluteAngle() < wristMinAngle) {
            volts = clamp(volts, -Double.MAX_VALUE, 0);
        }
        if (getAbsoluteAngle() > wristMaxAngle) {
            volts = clamp(volts, 0, Double.MAX_VALUE);
        }

        motor.setVoltage(volts);
    }

    /**
     * Interpolates our PID and feedforward constants and sets them.
     * @param elevatorPosMet Elevator current position in meters
     * @param pivotAngleDeg Pivot current angle in degrees
     */
    private void interpolateConstants(double elevatorPosMet, double pivotAngleDeg) {
        //TODO either set this to bangbang controller or calculate the lagrange as this 
        // is not a simple function
    wristPIDController.setPID(
        0, 
        0, 
        0);
        FeedForwardKs = 0.0 * pivotAngleDeg * elevatorPosMet;
        FeedForwardKv = 0.0 * pivotAngleDeg * elevatorPosMet;
        FeedForwardKa = 0.0 * pivotAngleDeg * elevatorPosMet;
    }

    /**
     * @return the current rotation of the wrist, measured in degrees.
     */
    private double getAbsoluteAngle() {
        return (wristEncoder.get() * 360) - encoderOffset;
    }

    /**
     * 
     * @return the current velocity of the wrist in degrees
     */
    private double getWristVelocity() {
        return (motor.getVelocity().getValueAsDouble());
    }

    /**
     * 
     * @param velocity Current velocity of wrist in degrees per second
     * @param acceleration Current acceleration of wrist in degrees per second squared
     * @return Our calculated simple feedforward values
     */
    private double calculateWristFeedForward(double velocity, double acceleration) {
        return 
        FeedForwardKs * Math.signum(velocity) 
        + FeedForwardKv * velocity 
        + FeedForwardKa * acceleration;
    }
    
    @Override
    public void setAngle(double goalAngleDeg, double elevatorPosMet, double pivotAngleDeg) {
        goalAngleDeg = clamp(goalAngleDeg, wristMinAngle, wristMaxAngle);
        interpolateConstants(elevatorPosMet, pivotAngleDeg);

        double acceleration = (wristPIDController.getSetpoint().velocity - lastVelocity) / (Timer.getFPGATimestamp() - lastTime);

        double pidOutput = wristPIDController.calculate(
            getAbsoluteAngle(), 
            goalAngleDeg);
        double feedForward = calculateWristFeedForward(
            wristPIDController.getGoal().velocity, 
            acceleration);

        Logger.recordOutput("Wrist/CalculatedFFVolts", feedForward);
        Logger.recordOutput("Wrist/PIDCommandVolts", pidOutput);

        Logger.recordOutput("Wrist/VelocityError", wristPIDController.getVelocityError());
        Logger.recordOutput("Wrist/PositionError", wristPIDController.getPositionError());

        setVoltage(feedForward + pidOutput);

        lastVelocity = wristPIDController.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();
    }

    @Override 
    public void setVoltage(double volts) {
        OpenLoopStatus = true;
        setMotorVoltage(volts);
    }

    @Override
    public void wristResetProfile() {
        wristPIDController.reset(getAbsoluteAngle(), getWristVelocity());
    }
}
