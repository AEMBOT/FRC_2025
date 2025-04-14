package frc.robot.subsystems.elevator;

import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.wpilibj.Timer.delay;
import static frc.robot.constants.ElevatorConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX leadingMotor = new TalonFX(TOP_MOTOR_ID);
  private final TalonFX followingMotor = new TalonFX(BOTTOM_MOTOR_ID);
  private double elevatorGoal;
  private double motorOffset;
  private double maxExtension;

  public ElevatorIOReal() {
    TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

    leftMotorConfig.CurrentLimits.StatorCurrentLimit = TOP_MOTOR_CURRENT_LIMIT;
    leftMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    rightMotorConfig.CurrentLimits.StatorCurrentLimit = BOTTOM_MOTOR_CURRENT_LIMIT;
    rightMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    leftMotorConfig.Slot0.kG = 0.35;
    leftMotorConfig.Slot0.kS = 0.15;
    leftMotorConfig.Slot0.kV = 0;
    leftMotorConfig.Slot0.kA = 0;
    leftMotorConfig.Slot0.kP = 9;
    leftMotorConfig.Slot0.kI = 0;
    leftMotorConfig.Slot0.kD = 0;

    leftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 4 / rotToMetMultFactor;
    leftMotorConfig.MotionMagic.MotionMagicAcceleration = 4 / rotToMetMultFactor;
    leftMotorConfig.MotionMagic.MotionMagicJerk = 0 / rotToMetMultFactor;

    leadingMotor.getConfigurator().apply(leftMotorConfig);
    followingMotor.getConfigurator().apply(rightMotorConfig);

    leadingMotor.setNeutralMode(NeutralModeValue.Brake);
    followingMotor.setNeutralMode(NeutralModeValue.Brake);

    followingMotor.setControl(new Follower(TOP_MOTOR_ID, TOP_MOTOR_INVERTED));

    elevatorGoal = 0;

    delay(1);

    motorOffset = -getAbsoluteMotorPosition();

    maxExtension = MAX_HEIGHT;
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    Logger.recordOutput("Elevator/maxExtension", maxExtension);
    inputs.elevatorAbsolutePosition = getAbsoluteMotorPosition();
    inputs.elevatorAbsoluteVelocity =
        leadingMotor.getVelocity().getValueAsDouble() * rotToMetMultFactor;
    inputs.elevatorAppliedVolts = leadingMotor.getMotorVoltage().getValueAsDouble();
    inputs.elevatorCurrentAmps =
        new double[] {
          leadingMotor.getStatorCurrent().getValueAsDouble(),
          followingMotor.getStatorCurrent().getValueAsDouble()
        };
    inputs.elevatorGoalPosition = elevatorGoal;
  }

  /**
   * Get our absolute motor position in meters.
   *
   * @return Absolute motor position in meters.
   */
  private double getAbsoluteMotorPosition() {
    return (leadingMotor.getPosition().getValueAsDouble() * rotToMetMultFactor) + motorOffset;
  }

  /**
   * Take our volts and then clamp them if we are at our min/max height.
   *
   * @param volts Volts to feed in.
   */
  private void setMotorVoltage(double volts) {
    if (getAbsoluteMotorPosition() < MIN_HEIGHT) {
      volts = clamp(volts, 0, Double.MAX_VALUE);
    }
    if (getAbsoluteMotorPosition() > MAX_HEIGHT) {
      volts = clamp(volts, -Double.MAX_VALUE, 0);
    }
    leadingMotor.setVoltage(-volts);
  }

  @Override
  public void limitHeight(double pivotAngle) {

    Logger.recordOutput("i", pivotAngle);
    maxExtension =
        (pivotAngle > 90)
            ? Units.inchesToMeters(20) / Math.cos(Units.degreesToRadians(180 - pivotAngle))
                - Units.inchesToMeters(39)
            : Units.inchesToMeters(35) / Math.cos(Units.degreesToRadians(pivotAngle))
                - Units.inchesToMeters(39);
    if (getAbsoluteMotorPosition() > maxExtension) {
      setHeight(maxExtension);
    }
  }

  @Override
  public void setHeight(double height) {

    height = clamp(height, MIN_HEIGHT, MAX_HEIGHT);
    height = clamp(height, MIN_HEIGHT, maxExtension);

    elevatorGoal = height;
    final MotionMagicVoltage request =
        new MotionMagicVoltage((height - motorOffset) / rotToMetMultFactor);
    leadingMotor.setControl(request);
  }

  @Override
  public void setVoltage(double volts) {
    setMotorVoltage(volts);
  }

  @Override
  public void resetProfile() {
    elevatorGoal = getAbsoluteMotorPosition();
  }

  @Override
  public void setMotorZero() {
    motorOffset = -leadingMotor.getPosition().getValueAsDouble();
  }
}
