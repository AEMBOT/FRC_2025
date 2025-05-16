package frc.robot.subsystems.wrist;

import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.wpilibj.Timer.delay;
import static frc.robot.constants.WristConstants.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.littletonrobotics.junction.Logger;

public class WristIOReal implements WristIO {

  private boolean openLoop = true;
  private final TalonFX motor = new TalonFX(MOTOR_ID);

  private double wristGoal;
  private double rotorOffset;

  public WristIOReal() {
    delay(3);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.CurrentLimits.StatorCurrentLimit = MOTOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motorConfig.Slot0.kP = 10;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 2500;
    motorConfig.MotionMagic.MotionMagicAcceleration = 5000;

    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motorConfig.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / MOTOR_RATIO));

    setMotorZero();

    motor.getConfigurator().apply(motorConfig);

    motor.setNeutralMode(NeutralModeValue.Brake);

    wristGoal = getAbsoluteMotorPosition();
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAbsolutePosition = getAbsoluteMotorPosition();
    Logger.recordOutput("Wrist/motorTemp", motor.getDeviceTemp().getValueAsDouble());
    Logger.recordOutput("Wrist/rawMotorRotations", motor.getPosition().getValueAsDouble());
    Logger.recordOutput(
        "Wrist/directMotorValue",
        360 * ((motor.getPosition().getValueAsDouble() - rotorOffset)));
    inputs.wristAbsoluteVelocity = motor.getVelocity().getValueAsDouble();
    inputs.wristAppliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.wristCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.wristGoalPosition = wristGoal;
    inputs.openLoopStatus = openLoop;
  }

  @Override
  public void setAngle(double angle) {
    angle = clamp(angle, MIN_ANGLE, MAX_ANGLE);
    wristGoal = angle;

    final MotionMagicVoltage request =
        new MotionMagicVoltage((MOTOR_RATIO * ((angle) / 360)) - rotorOffset);
    motor.setControl(request);
  }

  @Override
  public void setVoltage(double volts) {
    openLoop = true;
    setMotorVoltage(volts);
  }

  private double getAbsoluteMotorPosition() {
    return (((motor.getPosition().getValueAsDouble() + rotorOffset)) * 360);
  }

  private void setMotorVoltage(double volts) {

    if (getAbsoluteMotorPosition() < MIN_ANGLE) {
      volts = clamp(volts, 0, Double.MAX_VALUE);
    }
    if (getAbsoluteMotorPosition() > MAX_ANGLE) {
      volts = clamp(volts, -Double.MAX_VALUE, 0);
    }

    motor.setVoltage(-volts);
  }

  @Override
  public void setMotorZero() {
    rotorOffset = (ZERO_POSITION / 360) - motor.getPosition().getValueAsDouble();
  }

  @Override
  public void resetProfile() {
    setAngle(getAbsoluteMotorPosition());
  }
}
