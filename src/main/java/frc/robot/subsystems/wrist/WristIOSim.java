package frc.robot.subsystems.wrist;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.constants.SimConstants.*;
import static frc.robot.subsystems.pivot.PivotIOSim.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class WristIOSim implements WristIO {
  private double LAST_DISTANCE;
  private double LAST_TIME;

  private final Encoder m_encoder = new Encoder(12, 13);
  private final TrapezoidProfile.Constraints m_wristProfile =
      new TrapezoidProfile.Constraints(100, 250);
  private final ProfiledPIDController m_wristController =
      new ProfiledPIDController(0.1, 0, 0, m_wristProfile, 0.02);

  private final EncoderSim m_wristEncoderSim = new EncoderSim(m_encoder);
  private final MechanismLigament2d m_wristMech;

  public WristIOSim() {
    m_wristMech =
        m_pivotMech.append(
            new MechanismLigament2d("WristMech", 1, 0, 10, new Color8Bit(Color.kPlum)));
    SmartDashboard.putData("ArmMech", m_pivotCanvas);
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.wristAbsolutePosition = getAbsoluteEncoderPosition();
    inputs.wristAbsoluteVelocity = getAbsoluteEncoderVelocity();
    inputs.wristAppliedVolts = wristSim.getInput(0);
  }

  private double getAbsoluteEncoderPosition() {
    return m_wristEncoderSim.getDistance();
  }

  private double getAbsoluteEncoderVelocity() {
    var calcEncoderVelocity =
        ((m_wristEncoderSim.getDistance() - LAST_DISTANCE)
            / (Timer.getFPGATimestamp() - LAST_TIME));
    LAST_DISTANCE = m_wristEncoderSim.getDistance();
    LAST_TIME = Timer.getFPGATimestamp();
    return calcEncoderVelocity;
  }

  @Override
  public void setAngle(double angle) {
    angle = clamp(angle, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE);

    double pidOutput = m_wristController.calculate(getAbsoluteEncoderPosition(), angle);

    setVoltage(pidOutput);
  }

  @Override
  public void setVoltage(double volts) {
    setMotorVoltage(volts);
  }

  private void setMotorVoltage(double volts) {
    wristSim.setInput(volts);
  }

  @Override
  public void simulationPeriodic() {
    wristSim.update(0.02);

    m_wristEncoderSim.setDistance(Units.radiansToDegrees(wristSim.getAngleRads()));

    m_wristMech.setAngle(Units.radiansToDegrees(wristSim.getAngleRads()));
  }

  @Override
  public void resetProfile() {
    m_wristController.reset(getAbsoluteEncoderPosition());
  }
}
