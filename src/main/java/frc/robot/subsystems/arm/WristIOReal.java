package frc.robot.subsystems.arm;

import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.wpilibj.Timer.delay;
import static frc.robot.constants.WristConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class WristIOReal implements WristIO {

  private boolean OpenLoopStatus = true;
  private final TalonFX motor = new TalonFX(motorID);

  private double lastTime;
  private double lastVelocity;

  // Wrist FF variables
  private double FeedForwardKs = wristFFValues[0];
  private double FeedForwardKg = wristFFValues[1];
  private double FeedForwardKv = wristFFValues[2];
  private double FeedForwardKa = wristFFValues[3];

  public WristIOReal() {

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.StatorCurrentLimit = wristMotorCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(motorConfig);

    motor.setNeutralMode(NeutralModeValue.Brake);

    while (getAbsoluteAngleDeg() < -120 || getAbsoluteAngleDeg() > 120) {
      // TODO Look into better solutions for invalid encoder initial pose
      // TODO change min and max
      System.out.println(
          "ERROR: Busyloop because wrist position invalid! Put in valid position! Current angle: "
              + getAbsoluteAngleDeg());
      delay(1);
    }
    // for our at GOAL and at SETPOINT booleans
    wristPIDController.setTolerance(wristAngleToleranceDeg, wristVelocityTolerangeDegPerSec);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAbsAngle = getAbsoluteAngleDeg();
    inputs.wristRelativeMotorAngle = getRelativeMotorAngleDeg();
    inputs.wristMotorVelocity = getWristMotorVelocityDeg();
    inputs.wristGoal = wristPIDController.getGoal().position;
    inputs.wristSetpoint = wristPIDController.getSetpoint().position;
    inputs.wristAppliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.wristCurrentAmps = motor.getStatorCurrent().getValueAsDouble();

    inputs.wristOpenLoopStatus = OpenLoopStatus;
    inputs.wristAtGoal = wristPIDController.atGoal();
    inputs.wristAtSetpoint = wristPIDController.atSetpoint();
  }

  /**
   * Sets the voltage of the wrist motor.
   *
   * @param volts The voltage to set.
   */
  private void setMotorVoltage(double volts) {
    OpenLoopStatus = true;
    if (getAbsoluteAngleDeg() < wristMinAngle) {
      volts = clamp(volts, 0, Double.MAX_VALUE);
    }
    if (getAbsoluteAngleDeg() > wristMaxAngle) {
      volts = clamp(volts, -Double.MAX_VALUE, 0);
    }

    motor.setVoltage(volts);
  }

  /**
   * Interpolates our PID and feedforward constants and sets them.
   *
   * @param elevatorPosMet Elevator current position in meters
   * @param pivotAngleDeg Pivot current angle in degrees
   */
  private void interpolateConstants(double elevatorPosMet, double pivotAngleDeg) {
    //    wristPIDController.setPID(
    //        0 * WristPIDFactor,
    //        0 * WristPIDFactor,
    //        0 * WristPIDFactor);
    //    FeedForwardKs = 0.0 * pivotAngleDeg * elevatorPosMet * WristFFactor;
    //    FeedForwardKv = 0.0 * pivotAngleDeg * elevatorPosMet * WristFFactor;
    //    FeedForwardKa = 0.0 * pivotAngleDeg * elevatorPosMet * WristFFactor;
  }

  /**
   * @return the current rotation of the wrist, measured in degrees.
   */
  private double getAbsoluteAngleDeg() {
    return (wristEncoder.get() * 360) + encoderOffset;
  }

  private double getRelativeMotorAngleDeg() {
    return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
  }

  /**
   * @return the current velocity of the wrist in degrees
   */
  private double getWristMotorVelocityDeg() {
    return Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble());
  }

  /**
   * @param positionRad Current position of wrist encoder in radians
   * @param velocityRadPerSec Current velocity of wrist in radians per second
   * @param accelerationRadPerSecSquared Current acceleration of wrist in radians per second squared
   * @return Our calculated arm feedforward values
   */
  private double calculateWristFeedForward(
      double positionRad, double velocityRadPerSec, double accelRadPerSecSquared) {
    return FeedForwardKs * Math.signum(velocityRadPerSec)
        + FeedForwardKg * Math.cos(positionRad)
        + FeedForwardKv * (velocityRadPerSec)
        + FeedForwardKa * (accelRadPerSecSquared);
  }

  @Override
  public void setAngle(double goalAngleDeg, double elevatorPosMet, double pivotAngleDeg) {
    goalAngleDeg = clamp(goalAngleDeg, wristMinAngle, wristMaxAngle);
    interpolateConstants(elevatorPosMet, pivotAngleDeg);

    double pidOutput =
        wristPIDController.calculate( // in radians
            Units.degreesToRadians(getAbsoluteAngleDeg()), Units.degreesToRadians(goalAngleDeg));

    double acceleration =
        ( // in radians
            wristPIDController.getSetpoint().velocity - lastVelocity)
            / (Timer.getFPGATimestamp() - lastTime);

    double feedForward =
        calculateWristFeedForward( // in radians
            wristPIDController.getSetpoint().position,
            wristPIDController.getSetpoint().velocity,
            acceleration);

    Logger.recordOutput("Wrist/CalculatedFFVolts", feedForward);
    Logger.recordOutput("Wrist/PIDFeedbackVolts", pidOutput);

    Logger.recordOutput(
        "Wrist/VelocityErrorDeg", Units.radiansToDegrees(wristPIDController.getVelocityError()));
    Logger.recordOutput(
        "Wrist/PositionErrorDeg", Units.radiansToDegrees(wristPIDController.getPositionError()));

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
    wristPIDController.reset(Units.degreesToRadians(getAbsoluteAngleDeg()), 0);
  }
}
