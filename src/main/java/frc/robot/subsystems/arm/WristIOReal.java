package frc.robot.subsystems.arm;

import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.wpilibj.Timer.delay;
import static frc.robot.Constants.WristConstants.*;

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
  private double FeedForwardKv = wristFFValues[1];
  private double FeedForwardKa = wristFFValues[2];

  private double theoreticalVolts;

  public WristIOReal() {

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.StatorCurrentLimit = wristMotorCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(motorConfig);

    motor.setNeutralMode(NeutralModeValue.Brake);

    while (getAbsoluteAngleDeg() < -90 || getAbsoluteAngleDeg() > 90) {
      // TODO Look into better solutions for invalid encoder initial pose
      System.out.println(
          "ERROR: Busyloop because wrist position invalid! Put in valid position! Current angle: "
              + getAbsoluteAngleDeg());
      delay(1);
    }

    wristPIDController.setTolerance(wristAngleToleranceDeg, wristVelocityTolerangeDegPerSec);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAbsAngle = getAbsoluteAngleDeg();
    inputs.wristRelativeAngle = getRelativeAngleDeg();
    inputs.wristVelocity = getWristVelocityDeg();
    inputs.wristGoal = wristPIDController.getGoal().position;
    inputs.wristSetpoint = wristPIDController.getSetpoint().position;
    inputs.wristAppliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.wristCurrentAmps = motor.getStatorCurrent().getValueAsDouble();

    inputs.wristOpenLoopStatus = OpenLoopStatus;
    inputs.wristAtGoal = wristPIDController.atGoal();
    inputs.wristAtSetpoint = wristPIDController.atSetpoint();

    inputs.wristTheoreticalVolts = theoreticalVolts;
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

    motor.setVoltage(-volts);
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
  ;

  /**
   * @return the current rotation of the wrist, measured in degrees.
   */
  private double getAbsoluteAngleDeg() {
    return Units.rotationsToDegrees(
        wristEncoder.get() - encoderOffset); // encoder offset in rotations
  }

  private double getRelativeAngleDeg() {
    return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
  }

  /**
   * @return the current velocity of the wrist in degrees
   */
  private double getWristVelocityDeg() {
    return Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble());
  }

  /**
   * @param velocity Current velocity of wrist in degrees per second
   * @param acceleration Current acceleration of wrist in degrees per second squared
   * @return Our calculated simple feedforward values
   */
  private double calculateWristFeedForward(double velocity, double acceleration) {
    return FeedForwardKs * Math.signum(velocity)
        + FeedForwardKv * velocity
        + FeedForwardKa * acceleration;
  }

  @Override
  public void setAngle(double goalAngleDeg, double elevatorPosMet, double pivotAngleDeg) {
    goalAngleDeg = clamp(goalAngleDeg, wristMinAngle, wristMaxAngle);
    interpolateConstants(elevatorPosMet, pivotAngleDeg);

    double pidOutput =
        wristPIDController.calculate( // in degrees
            getAbsoluteAngleDeg(), goalAngleDeg);

    double acceleration =
        ( // in degrees
            wristPIDController.getSetpoint().velocity - lastVelocity)
            / (Timer.getFPGATimestamp() - lastTime);

    double feedForward =
        calculateWristFeedForward( // in degrees
            wristPIDController.getSetpoint().velocity, acceleration);

    Logger.recordOutput("Wrist/CalculatedFFVolts", feedForward);
    Logger.recordOutput("Wrist/PIDFeedbackVolts", pidOutput);

    Logger.recordOutput("Wrist/VelocityErrorDeg", wristPIDController.getVelocityError());
    Logger.recordOutput("Wrist/PositionErrorDeg", wristPIDController.getPositionError());

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
    wristPIDController.reset(getAbsoluteAngleDeg(), 0);
  }
}
