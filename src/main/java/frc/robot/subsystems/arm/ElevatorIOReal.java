package frc.robot.subsystems.arm;

import static edu.wpi.first.wpilibj.Timer.delay;
import static frc.robot.constants.ElevatorConstants.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX leaderMotor = new TalonFX(leftMotorID);
  private final TalonFX followerMotor = new TalonFX(rightMotorID);
  private double leaderMotorMax = 20000;
  private double leaderMotorMin = 0;
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

    var leaderMotorOutputConfig = new MotorOutputConfigs();
    var leaderMotorTalonConfig = new TalonFXConfiguration();
    leaderMotorTalonConfig.CurrentLimits.StatorCurrentLimit = 20;
    // elevatorCurrentLimit;
    leaderMotorTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    leaderMotor.getConfigurator().apply(leaderMotorTalonConfig);

    // var followerMotorTalonConfig = new TalonFXConfiguration();
    // followerMotorTalonConfig.CurrentLimits.StatorCurrentLimit = elevatorCurrentLimit;
    // followerMotorTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;;

    followerMotor.getConfigurator().apply(leaderMotorTalonConfig);

    leaderMotor.setNeutralMode(NeutralModeValue.Brake);
    followerMotor.setNeutralMode(NeutralModeValue.Brake);

    followerMotor.setControl(new Follower(leftMotorID, false));

    elevatorPIDController.setTolerance(
        elevatorPositionToleranceMet, elevatorVelocityToleranceMetPerSec);

    delay(1);

    encoderOffset = -getElevatorPosition();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorLeftMotorRotationDeg = Units.degreesToRotations(getLeaderMotorRotation());
    inputs.elevatorRightMotorRotationDeg = followerMotor.getPosition().getValueAsDouble();
    inputs.elevatorLeftVoltage = leaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.elevatorRightVoltage = followerMotor.getMotorVoltage().getValueAsDouble();
    inputs.elevatorLeftMotorVelocityDegrees = Units.degreesToRotations(getLeaderMotorVelocity());
    inputs.elevatorRightMotorVelocityDegrees =
        Units.rotationsToDegrees(followerMotor.getVelocity().getValueAsDouble());
    inputs.elevatorVelocityMeters = getElevatorVelocity();
    inputs.elevatorPositionMet = getElevatorPosition();
    inputs.elevatorAtGoal = elevatorPIDController.atGoal();
    inputs.elevatorAtSetpoint = elevatorPIDController.atSetpoint();
    inputs.elevatorGoalPositionMet = elevatorPIDController.getGoal().position;
    inputs.elevatorOpenLoopStatus = openLoopStatus;
    inputs.elevatorCurrentDraw = leaderMotor.getStatorCurrent().getValueAsDouble();
    inputs.elevatorMaxPos = leaderMotorMax;
    inputs.elevatorMinPos = leaderMotorMin;
    inputs.isHoming = isHoming;
  }

  /**
   * Returns, in degrees, the position of the elevator leaderMotor relative to its starting
   * position.
   */
  private double getLeaderMotorRotation() {
    return Units.rotationsToDegrees(leaderMotor.getPosition().getValueAsDouble());
  }

  /** Returns (in degrees per second) the current velocity of the leaderMotor */
  private double getLeaderMotorVelocity() {
    return Units.rotationsToDegrees(leaderMotor.getVelocity().getValueAsDouble());
  }

  /** Returns (in meters per second) the current velocity of the elevator */
  private double getElevatorVelocity() {
    return leaderMotor.getVelocity().getValueAsDouble() * PositionFactor;
  }

  /** Returns (in meters) the position of the elevator relative to the start position */
  // TODO Before merge, make a way to get and set the absolute vertical position of the elevator
  private double getElevatorPosition() {
    return (leaderMotor.getPosition().getValueAsDouble() * PositionFactor) + encoderOffset;
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
  public void runCharacterizationVolts(double voltage) {
    leaderMotor.setVoltage(voltage);
  }

  @Override
  public void setVoltage(double voltage) {
    openLoopStatus = true;

    if (leaderMotor.getStatorCurrent().getValueAsDouble() > 100) {
      if (voltage > 0) {
        leaderMotorMax = getLeaderMotorRotation();
      } else if (voltage != 0) {
        leaderMotorMin = getLeaderMotorRotation();
      }
    }

    // if (getElevatorPosition() < elevatorMinPosMet) {
    //     voltage = clamp(voltage, 0, Double.MAX_VALUE);
    //   }
    //   if (getElevatorPosition() > elevatorMaxPosMet) {
    //     voltage = clamp(voltage, -Double.MAX_VALUE, 0);
    //   }

    if (Math.abs(getLeaderMotorRotation() - leaderMotorMax) <= 250 && voltage > 0) {
      leaderMotor.setVoltage(0);
      openLoopStatus = false;
    } else if (Math.abs(getLeaderMotorRotation() - leaderMotorMin) <= 250 && voltage < 0) {
      leaderMotor.setVoltage(0);
      openLoopStatus = false;
    } else {
      leaderMotor.setVoltage(voltage);
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

    leaderMotor.setVoltage(pidOutput + feedForward);

    lastVelocity = elevatorPIDController.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void elevatorResetProfile() {
    elevatorPIDController.reset(getElevatorPosition(), 0);
  }

  @Override
  public boolean atMinimum() {
    // what is this
    return (Math.round(getLeaderMotorRotation() / 10) * 10
            == Math.round(getLeaderMotorRotation() / 10) * 10)
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
