package frc.robot.subsystems.pivot;

import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.wpilibj.Timer.delay;
import static frc.robot.constants.PivotConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.littletonrobotics.junction.Logger;

public class PivotIOReal implements PivotIO {

  private boolean openLoop = true;
  private final TalonFX leadingMotor = new TalonFX(LEFT_MOTOR_ID);
  private final TalonFX followingMotor = new TalonFX(RIGHT_MOTOR_ID);
  private final DigitalOutput ratchetPin1;
  private final DigitalOutput ratchetPin2;
  private double pivotGoal;
  private TrapezoidProfile.State pivotSetpoint;
  private double lastTime;
  private TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
  private DutyCycleEncoder ENCODER;
  private double rotorOffset;
  private boolean ratchetEngaged;

  public PivotIOReal() {

    ENCODER = new DutyCycleEncoder(ENCODER_ID);

    delay(3);

    TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

    leftMotorConfig.CurrentLimits.StatorCurrentLimit = LEFT_MOTOR_CURRENT_LIMIT;
    leftMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    rightMotorConfig.CurrentLimits.StatorCurrentLimit = RIGHT_MOTOR_CURRENT_LIMIT;
    rightMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leftMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    leftMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    Logger.recordOutput("Pivot/StartingAbsEncoderValue", getAbsoluteEncoderPosition());
    rotorOffset =
        (GEAR_RATIO * getAbsoluteEncoderPosition() / 360)
            - leadingMotor.getPosition().getValueAsDouble();

    leftMotorConfig.Feedback.FeedbackRotorOffset = rotorOffset;
    Logger.recordOutput(
        "Pivot/MotorEncoderWithOffset",
        leadingMotor.getPosition().getValueAsDouble() + rotorOffset);
    Logger.recordOutput("Pivot/rotorOffset", rotorOffset);
    Logger.recordOutput(
        "Pivot/reverseCalculatedPosition",
        ((rotorOffset + leadingMotor.getPosition().getValueAsDouble()) / GEAR_RATIO) * 360);

    leftMotorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;

    // Integrating CANcoder
    /* Configure CANcoder to zero the magnet appropriately */

    // CANcoderConfiguration cc_cfg = new CANcoderConfiguration();

    // cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    // cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    // cc_cfg.MagnetSensor.MagnetOffset = 0.4;

    // m_cc.getConfigurator().apply(cc_cfg); Dont know what m_cc is but hey its here

    // Apply cancoder to feedback motor config
    // leftMotorConfig.Feedback.FeedbackRemoteSensorID = 1;
    // leftMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // leftMotorConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
    // leftMotorConfig.Feedback.FeedbackRotorOffset = 0;

    // leftMotorConfig.Slot0.kG = -0.3;
    // leftMotorConfig.Slot0.kS = -1;
    // leftMotorConfig.Slot0.kV = 0;
    // leftMotorConfig.Slot0.kA = 0;
    leftMotorConfig.Slot0.kP = 1;
    leftMotorConfig.Slot0.kI = 0;
    leftMotorConfig.Slot0.kD = 0;

    leftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 25;
    leftMotorConfig.MotionMagic.MotionMagicAcceleration = 50;
    leftMotorConfig.MotionMagic.MotionMagicJerk = 0;

    leadingMotor.getConfigurator().apply(leftMotorConfig);
    followingMotor.getConfigurator().apply(rightMotorConfig);

    leadingMotor.setNeutralMode(NeutralModeValue.Brake);
    followingMotor.setNeutralMode(NeutralModeValue.Brake);

    followingMotor.setControl(new Follower(LEFT_MOTOR_ID, true));

    /**
     * while (getAbsoluteEncoderPosition() < MIN_ANGLE || getAbsoluteEncoderPosition() > MAX_ANGLE)
     * { // TODO Look into better solutions for invalid encoder initial pose
     * System.out.println("ERROR: Busyloop because pivot position invalid! Is the encoder plugged
     * in?"); delay(1); }
     */
    pivotGoal = getAbsoluteEncoderPosition();
    pivotSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);

    ratchetPin1 = new DigitalOutput(RATCHET_PIN_1_ID);
    ratchetPin2 = new DigitalOutput(RATCHET_PIN_2_ID);
  }

  public void updateInputs(PivotIOInputs inputs) {
    Logger.recordOutput(
        "Pivot/absoluteMotorPosition",
        (((leadingMotor.getPosition().getValueAsDouble() + rotorOffset) / GEAR_RATIO) * 360));
    inputs.pivotAbsolutePosition = getAbsoluteEncoderPosition();
    inputs.pivotAbsoluteVelocity = leadingMotor.getVelocity().getValueAsDouble();
    inputs.pivotAppliedVolts = leadingMotor.getMotorVoltage().getValueAsDouble();
    inputs.pivotCurrentAmps =
        new double[] {
          leadingMotor.getStatorCurrent().getValueAsDouble(),
          followingMotor.getStatorCurrent().getValueAsDouble()
        };
    inputs.pivotPosition = pivotGoal;
    inputs.pivotSetpointPosition = pivotSetpoint.position;
    inputs.pivotSetpointVelocity = pivotSetpoint.velocity;
    inputs.openLoopStatus = openLoop;
    inputs.ratchetEngaged = ratchetEngaged;
  }

  @Override
  public void setAngle(double angle) {

    angle = clamp(angle, MIN_ANGLE, MAX_ANGLE);

    pivotGoal = angle;
    final MotionMagicVoltage request =
        new MotionMagicVoltage((GEAR_RATIO * ((angle) / 360)) - rotorOffset);
    leadingMotor.setControl(request);
  }

  @Override
  public void setVoltage(double volts) {
    openLoop = true;
    setMotorVoltage(volts);
  }

  private double getAbsoluteEncoderPosition() {
    return ((ENCODER.get() * 360) + ENCODER_POSITION_OFFSET) % 360;
  }

  private void setMotorVoltage(double volts) {

    if (getAbsoluteEncoderPosition() < MIN_ANGLE) {
      volts = clamp(volts, 0, Double.MAX_VALUE);
    }
    if (getAbsoluteEncoderPosition() > MAX_ANGLE) {
      volts = clamp(volts, -Double.MAX_VALUE, 0);
    }

    leadingMotor.setVoltage(-volts);
  }

  @Override
  public void resetProfile() {
    pivotGoal = getAbsoluteEncoderPosition();
    pivotSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
  }

  @Override
  public void runRatchetForward() {
    ratchetPin1.set(false);
    ratchetPin2.set(true);
    ratchetEngaged = true;
  }

  @Override
  public void runRatchetReverse() {
    ratchetPin1.set(true);
    ratchetPin2.set(false);
    ratchetEngaged = false;
  }

  @Override
  public void stopRatchet() {
    ratchetPin1.set(true);
    ratchetPin2.set(true);
  }
}
