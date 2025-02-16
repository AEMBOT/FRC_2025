package frc.robot.subsystems.arm;

import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.wpilibj.Timer.delay;
import static frc.robot.Constants.UPDATE_PERIOD;
import static frc.robot.Constants.WristConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class WristIOReal implements WristIO {
  private final TalonFX motor = new TalonFX(motorID);
  private TrapezoidProfile.State wristGoal;
  private TrapezoidProfile.State wristSetpoint;
  private double theoreticalVolts;

  public WristIOReal() {

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.StatorCurrentLimit = wristMotorCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(motorConfig);

    motor.setNeutralMode(NeutralModeValue.Brake);

    while (getAbsoluteAngle() < -90
        || getAbsoluteAngle() > 90) { // TODO Get a better solution for this
      System.out.println(
          "Wrist in busyloop! Put in valid position! Current angle: " + getAbsoluteAngle());
      delay(1);
    }

    wristGoal = new TrapezoidProfile.State(getAbsoluteAngle(), 0);
    wristSetpoint = new TrapezoidProfile.State(getAbsoluteAngle(), 0);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAbsAngle = getAbsoluteAngle();
    inputs.wristAbsVelocity = motor.getVelocity().getValueAsDouble() * 360 / 45;
    inputs.wristGoal = wristGoal.position;
    inputs.wristSetpoint = wristSetpoint.position;
    inputs.wristAppliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.wristCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.wristTheoreticalVolts = theoreticalVolts;
  }

  @Override
  public void setAngle(Double angle) {
    angle = clamp(angle, wristMinAngle, wristMaxAngle);

    wristGoal = new TrapezoidProfile.State(angle, 0);

    wristSetpoint = wristProfile.calculate(UPDATE_PERIOD, wristSetpoint, wristGoal);

    theoreticalVolts = wristSetpoint.position;

    double feedForward = wristFFModel.calculate(wristGoal.position, 0);
    double pidOutput = wristPIDController.calculate(getAbsoluteAngle(), wristGoal.position);

    setVoltage(feedForward + pidOutput);
  }

  /**
   * Sets the voltage of the wrist motor.
   *
   * @param volts The voltage to set.
   */
  private void setVoltage(double volts) {
    if (getAbsoluteAngle() < wristMinAngle) {
      volts = clamp(volts, 0, Double.MAX_VALUE);
      System.out.println("Wrist is less than min angle");
    }
    if (getAbsoluteAngle() > wristMaxAngle) {
      volts = clamp(volts, -Double.MAX_VALUE, 0);
      System.out.println("Wrist is more than max angle.");
    }

    motor.setVoltage(-volts);
  }

  /**
   * @return the current rotation of the wrist, measured in degrees.
   */
  private double getAbsoluteAngle() {
    return (wristEncoder.get() * 360) + encoderOffset;
  }

  @Override
  public void resetProfile() {
    wristGoal = new TrapezoidProfile.State(getAbsoluteAngle(), 0);
    wristSetpoint = new TrapezoidProfile.State(getAbsoluteAngle(), 0);
  }
}
