package frc.robot.subsystems.arm;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.constants.ElevatorConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX motor = new TalonFX(motorID);
  private double motorMax = 20000;
  private double motorMin = 0;
  private boolean isHoming = false;
  private double encoderOffset = 0;

  private double lastTime;
  private double lastVelocity;
  private boolean openLoopStatus = true;

  // elevator FF values because built in class doesn't work
  private double FeedForwardKs = elevatorFFValues[0];
  private double FeedForwardKg = elevatorFFValues[1];
  private double FeedForwardKv = elevatorFFValues[2];
  private double FeedForwardKa = elevatorFFValues[3];

  public ElevatorIOReal() {

    var motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.StatorCurrentLimit = elevatorCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motor.setNeutralMode(NeutralModeValue.Brake);

    motor.getConfigurator().apply(motorConfig);

    elevatorPIDController.setTolerance(
        elevatorPositionToleranceMet, elevatorVelocityToleranceMetPerSec);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorMotorRotationDeg = getMotorRotation();
    inputs.elevatorVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.elevatorMotorVelocityDegrees = getMotorVelocity();
    inputs.elevatorVelocityMeters = getElevatorVelocity();
    inputs.elevatorPositionMet = getElevatorPosition();
    inputs.elevatorAtGoal = elevatorPIDController.atGoal();
    inputs.elevatorAtSetpoint = elevatorPIDController.atSetpoint();
    inputs.elevatorGoalPositionMet = elevatorPIDController.getGoal().position;
    inputs.elevatorOpenLoopStatus = openLoopStatus;
    inputs.elevatorCurrentDraw = motor.getStatorCurrent().getValueAsDouble();
    inputs.elevatorMaxPos = motorMax;
    inputs.elevatorMinPos = motorMin;
    inputs.isHoming = isHoming;
  }

  /** Returns, in degrees, the position of the elevator motor relative to its starting position. */
  private double getMotorRotation() {
    return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
  }

  /** Returns (in degrees per second) the current velocity of the motor */
  private double getMotorVelocity() {
    return Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble());
  }

  /** Returns (in meters per second) the current velocity of the elevator */
  private double getElevatorVelocity() {
    return Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble()) * PositionFactor;
  }

  /** Returns (in meters) the position of the elevator relative to the start position */
  // TODO Before merge, make a way to get and set the absolute vertical position of the elevator
  private double getElevatorPosition() {
    return clamp(
        Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) * PositionFactor, 0, 5);
  }

  /**
   * Manually calculates our feedforward values.
   *
   * @param velocity Current velocity of our elevator in meters per second.
   * @param acceleration Current acceleration of our elevator in meters per second squared.
   * @return Our feedforward values.
   */
  private double calculateElevatorFeedforward(double velocity, double acceleration) {
    return FeedForwardKs * Math.signum(velocity)
        + FeedForwardKg
        + FeedForwardKv * velocity
        + FeedForwardKa * acceleration;
  }

  /**
   * Interpolates our elevator PID and feedforward constants.
   *
   * @param pivotAngleDeg Current angle of our pivot in degrees.
   */
  private void interpolateConstants(double pivotAngleDeg) {
    // elevatorPIDController.setPID(
    //    0 * ElevatorPIDFactor,
    //    0 * ElevatorPIDFactor,
    //    0 * ElevatorPIDFactor);
    // FeedForwardKs = 0.0 * pivotAngleDeg * ElevatorFFactor;
    // FeedForwardKv = 0.0 * pivotAngleDeg * ElevatorFFactor;
    // FeedForwardKg = 0.0 * pivotAngleDeg * ElevatorFFactor;
    // FeedForwardKa = 0.0 * pivotAngleDeg * ElevatorFFactor;
  }

  @Override
  public void setVoltage(double voltage) {
    openLoopStatus = true;

    if (motor.getStatorCurrent().getValueAsDouble() > 100) {
      if (voltage > 0) {
        motorMax = getMotorRotation();
      } else if (voltage != 0) {
        motorMin = getMotorRotation();
      }
    }

    if (Math.abs(getMotorRotation() - motorMax) <= 250 && voltage > 0) {
      motor.setVoltage(0);
      openLoopStatus = false;
    } else if (Math.abs(getMotorRotation() - motorMin) <= 250 && voltage < 0) {
      motor.setVoltage(0);
      openLoopStatus = false;
    } else {
      motor.setVoltage(voltage);
    }
  }

  @Override
  public void setGoalPosition(double goalPosMet, double pivotAngleDeg) {
    openLoopStatus = false;
    interpolateConstants(pivotAngleDeg);

    double pidOutput = elevatorPIDController.calculate(getElevatorPosition(), goalPosMet);

    double acceleration =
        (elevatorPIDController.getSetpoint().velocity - lastVelocity) // in meters
            / (Timer.getFPGATimestamp() - lastTime);

    double feedForward =
        calculateElevatorFeedforward( // in meters
            elevatorPIDController.getSetpoint().velocity, acceleration);

    Logger.recordOutput("Elevator/FeedForwardVolts", feedForward);
    Logger.recordOutput("Elevator/FeedBackVolts", pidOutput);

    Logger.recordOutput(
        "Elevator/VelocityErrorMetersPerSec", elevatorPIDController.getVelocityError());
    Logger.recordOutput("Elevator/PositionErrorMeters", elevatorPIDController.getPositionError());

    motor.setVoltage(pidOutput + feedForward);

    lastVelocity = elevatorPIDController.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void elevatorResetProfile() {
    elevatorPIDController.reset(getElevatorPosition(), 0);
  }

  @Override
  public boolean atMinimum() {

    return (Math.round(getMotorRotation() / 10) * 10 == Math.round(getMotorRotation() / 10) * 10)
        ? false
        : true;
  }

  @Override
  public void setHoming(boolean homingValue) {
    isHoming = homingValue;
  }

  @Override
  public void setEncoder(double encoderValue) {}
}
