package frc.robot.subsystems.pivot;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.constants.SimConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class PivotIOSim implements PivotIO {
  // normal stuff
  private double LAST_DISTANCE = 0.0;
  private double LAST_TIME = 0.0;
  private final Encoder m_encoder = new Encoder(10, 11);
  private final TrapezoidProfile.Constraints m_pivotProfile =
      new TrapezoidProfile.Constraints(100, 250);
  private final ProfiledPIDController m_pivotController =
      new ProfiledPIDController(1.0, 0, 0, m_pivotProfile, 0.02);

  private final EncoderSim m_pivotEncoderSim = new EncoderSim(m_encoder);
  private final MechanismRoot2d m_pivotRoot;

  // use static so other sim can use this as root, sort of like constants except
  // it needs to be instantiated, so we don't use constant file
  public static final Mechanism2d m_pivotCanvas = new Mechanism2d(5, 10);
  public static MechanismLigament2d m_pivotMech;

  public PivotIOSim() {
    m_pivotRoot = m_pivotCanvas.getRoot("PivotRoot", 2, 0.1);
    m_pivotMech =
        m_pivotRoot.append(
            new MechanismLigament2d("PivotMech", 1, 0, 5, new Color8Bit(Color.kLavender)));
  }

  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotAbsolutePosition = getAbsoluteEncoderPosition();
    inputs.pivotAbsoluteVelocity = getAbsoluteEncoderVelocity();
    inputs.pivotAppliedVolts = pivotSim.getInput(0);
  }

  @Override
  public void setVoltage(double volts) {
    setMotorVoltage(volts);
  }

  @Override
  public void setAngle(double angle) {
    angle = clamp(angle, PIVOT_MIN_ANGLE, PIVOT_MAX_ANGLE);

    double pidOutput = m_pivotController.calculate(getAbsoluteEncoderPosition(), angle);

    setVoltage(pidOutput);
  }

  private double getAbsoluteEncoderPosition() {
    return (m_pivotEncoderSim.getDistance());
  }

  private double getAbsoluteEncoderVelocity() {
    var calcEncoderVelocity =
        ((m_pivotEncoderSim.getDistance() - LAST_DISTANCE)
            / (Timer.getFPGATimestamp() - LAST_TIME));
    LAST_DISTANCE = m_pivotEncoderSim.getDistance();
    LAST_TIME = Timer.getFPGATimestamp();
    return calcEncoderVelocity;
  }

  private void setMotorVoltage(double volts) {
    // if (getAbsoluteEncoderPosition() < MIN_ANGLE) {
    //   volts = clamp(volts, 0, Double.MAX_VALUE);
    // }
    // if (getAbsoluteEncoderPosition() > MAX_ANGLE) {
    //   volts = clamp(volts, -Double.MAX_VALUE, 0);
    // }

    pivotSim.setInput(volts);
  }

  @Override
  public void simulationPeriodic() {
    pivotSim.update(0.02);

    m_pivotEncoderSim.setDistance(Units.radiansToDegrees(pivotSim.getAngleRads()));

    m_pivotMech.setAngle(Units.radiansToDegrees(pivotSim.getAngleRads()));
  }

  @Override
  public void resetProfile() {
    m_pivotController.reset(getAbsoluteEncoderPosition());
  }
}
