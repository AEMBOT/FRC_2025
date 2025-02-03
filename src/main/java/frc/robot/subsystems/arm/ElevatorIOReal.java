package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorIOReal implements ElevatorIO { 
    private final TalonFX motor = new TalonFX(motorID);

    private double lastTime;
    private double lastVelocity;
    private boolean openLoopStatus = true;

    //elevator FF values because built in class doesn't work
    private double FeedForwardKs = 0;
    private double FeedForwardKg = 0;
    private double FeedForwardKv = 0;
    private double FeedForwardKa = 0;

    public ElevatorIOReal() {
        var motorConfig = new MotorOutputConfigs();
        motorConfig.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(motorConfig);

        //TODO set proper tolerance value for our PID controller
        elevatorPIDController.setTolerance(0.1, 0.1);
    }

    /* Returns, in degrees, the position of the elevator motor relative to its starting position. */
    private double getMotorRotation() {
        return motor.getPosition().getValueAsDouble() * 360;
    }

    /* Returns (in degrees per second) the current velocity of the motor */
    private double getMotorVelocity() {
        return motor.getVelocity().getValueAsDouble() * 360;
    }

    /* Returns (in meters per second) the current velocity of the elevator */
    private double getElevatorVelocity() {
        return motor.getVelocity().getValueAsDouble() * 360 * PositionFactor;
    }

    /* Returns (in meters) the position of the elevator relative to the start position */
    private double getElevatorPosition() {
    // TODO Before merge, make a way to get and set the absolute vertical position of the elevator
        return clamp(motor.getPosition().getValueAsDouble() * 360 * PositionFactor, 0, 5);
    }

    // TODO Find values for the interpolation of our feedforward and feedback values
    // I think it is linear for FF, PID may be a different form
    // If needed we could totallllly find out the lagrange
    private void interpolateConstants(double pivotAngleDeg) {
        elevatorPIDController.setPID(
            0, 
            0, 
            0
            );
        FeedForwardKs = 0.0 * pivotAngleDeg;
        FeedForwardKv = 0.0 * pivotAngleDeg;
        FeedForwardKg = 0.0 * pivotAngleDeg;
        FeedForwardKa = 0.0 * pivotAngleDeg;
    }

    //Manually calculate elevator feedforward
    private double calculateElevatorFeedforward(double velocity, double acceleration) {
        return FeedForwardKs * Math.signum(velocity) + FeedForwardKg + FeedForwardKv * velocity + FeedForwardKa * acceleration;
      }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorMotorRotation = getMotorRotation();
        inputs.elevatorVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.elevatorVelocity = getMotorVelocity();
        inputs.elevatorPosition = getElevatorPosition();
        inputs.elevatorAtGoal = elevatorPIDController.atGoal();
        inputs.elevatorGoalPosition = elevatorPIDController.getGoal().position;
        inputs.elevatorOpenLoopStatus = openLoopStatus;
    }

    @Override
    public void setVoltage(double voltage) {
        openLoopStatus = true;
        motor.setVoltage(voltage);
    }

    @Override
    public void setGoalPosition(double goalPosMet, double pivotAngleDeg) {
        openLoopStatus = false;
        interpolateConstants(pivotAngleDeg);

        double pidOutput = elevatorPIDController.calculate(getElevatorPosition(), goalPosMet);
        double acceleration = (elevatorPIDController.getSetpoint().velocity - lastVelocity) / (Timer.getFPGATimestamp() - lastTime);
        double feedForward = calculateElevatorFeedforward(elevatorPIDController.getSetpoint().velocity, acceleration);
        motor.setVoltage(
            pidOutput
            + feedForward);
        lastVelocity = elevatorPIDController.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();        

        Logger.recordOutput("Elevator/FeedForwardVolts", feedForward);
        Logger.recordOutput("Elevator/FeedBackVolts", pidOutput);

        Logger.recordOutput("Elevator/VelocityError", elevatorPIDController.getVelocityError());
        Logger.recordOutput("Elevator/PositionError", elevatorPIDController.getPositionError());
    }

    @Override
    public void elevatorResetProfile() {
        elevatorPIDController.reset(getElevatorPosition(), getElevatorVelocity());
    }
}