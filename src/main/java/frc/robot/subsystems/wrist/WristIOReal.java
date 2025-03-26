package frc.robot.subsystems.wrist;

import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.wpilibj.Timer.delay;
import static frc.robot.constants.WristConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.littletonrobotics.junction.Logger;

public class WristIOReal implements WristIO {
  private final TalonFX motor = new TalonFX(MOTOR_ID);

  private DutyCycleEncoder ENCODER;
  private double wristGoal;
  private double rotorOffset;

  public WristIOReal() {

    ENCODER = new DutyCycleEncoder(ENCODER_ID);

    delay(3);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.CurrentLimits.StatorCurrentLimit = MOTOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motorConfig.Slot0.kP = 10;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 2500 / MOTOR_RATIO;
    motorConfig.MotionMagic.MotionMagicAcceleration = 5000 / MOTOR_RATIO;

    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rotorOffset =
        (MOTOR_RATIO * getAbsoluteEncoderPosition() / 360) - motor.getPosition().getValueAsDouble();

    motor.getConfigurator().apply(motorConfig);

    motor.setNeutralMode(NeutralModeValue.Brake);

    /**
     * while (getAbsoluteEncoderPosition() < MIN_ANGLE || getAbsoluteEncoderPosition() > MAX_ANGLE)
     * { // todo Look into better solutions for invalid encoder initial pose
     * System.out.println("ERROR: Busyloop because wrist position invalid! Is the encoder plugged
     * in?"); delay(1); }
     */
    wristGoal = getAbsoluteMotorPosition();
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAbsolutePosition = getAbsoluteEncoderPosition();
    Logger.recordOutput("Wrist/motorTemp", motor.getDeviceTemp().getValueAsDouble());
    Logger.recordOutput("rawWristMotorValue", ENCODER.get());
    Logger.recordOutput(
        "Pivot/directMotorValue",
        360 * ((motor.getPosition().getValueAsDouble() - rotorOffset) / 6));
    inputs.wristAbsoluteVelocity = motor.getVelocity().getValueAsDouble();
    inputs.wristAppliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.wristCurrentAmps =
        new double[] {
          motor.getStatorCurrent().getValueAsDouble(), motor.getStatorCurrent().getValueAsDouble()
        };
    inputs.wristGoalPosition = wristGoal;
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
    setMotorVoltage(volts);
  }

  private double getAbsoluteMotorPosition() {
    return (((motor.getPosition().getValueAsDouble() + rotorOffset) / MOTOR_RATIO) * 360);
  }

  private double getAbsoluteEncoderPosition() {
    return ((((1 - ENCODER.get()) * 180) + ENCODER_POSITION_OFFSET + 90) % 360) - 90;
  }

  private void setMotorVoltage(double volts) {

    if (getAbsoluteEncoderPosition() < MIN_ANGLE) {
      volts = clamp(volts, 0, Double.MAX_VALUE);
    }
    if (getAbsoluteEncoderPosition() > MAX_ANGLE) {
      volts = clamp(volts, -Double.MAX_VALUE, 0);
    }

    motor.setVoltage(-volts);
  }

  @Override
  public void resetGoal() {
    wristGoal = getAbsoluteMotorPosition();
  }
}
