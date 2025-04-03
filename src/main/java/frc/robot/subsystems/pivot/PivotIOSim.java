package frc.robot.subsystems.pivot;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.constants.PivotConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class PivotIOSim implements PivotIO {
  private double pivotGoal;

  private final Encoder encoder = new Encoder(3, 4);
  private final TrapezoidProfile.Constraints pivotProfile =
      new TrapezoidProfile.Constraints(100, 250);
  private final ProfiledPIDController pivotController =
      new ProfiledPIDController(10, 0, 0, pivotProfile, 0.02);
  private final EncoderSim pivotEncoderSim = new EncoderSim(encoder);
  private final MechanismRoot2d pivotRoot;

  public static final Mechanism2d pivotCanvas = new Mechanism2d(5, 10);
  public static MechanismLigament2d pivotMech;

  public PivotIOSim() {
    pivotRoot = pivotCanvas.getRoot("PivotRoot", 2, 0.1);
    pivotMech =
        pivotRoot.append(
            new MechanismLigament2d("PivotMech", 1, 0, 12, new Color8Bit(Color.kLightBlue)));
  }

  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotAbsolutePosition = getAbsoluteEncoderPosition();
    inputs.pivotAbsoluteVelocity = SIM.getVelocityRadPerSec();
    inputs.pivotAppliedVolts = SIM.getInput(0);
    inputs.pivotPosition = pivotGoal;
  }

  @Override
  public void setVoltage(double volts) {
    setMotorVoltage(volts);
  }

  @Override
  public void setAngle(double angle) {
    angle = clamp(angle, MIN_ANGLE, MAX_ANGLE);
    pivotGoal = angle;

    double pidOutput = pivotController.calculate(getAbsoluteEncoderPosition(), angle);
    setVoltage(pidOutput);
  }

  private double getAbsoluteEncoderPosition() {
    return (pivotEncoderSim.getDistance());
  }

  private void setMotorVoltage(double volts) {
    SIM.setInput(volts);
  }

  @Override
  public void simulationPeriodic() {
    SIM.update(0.02);
    pivotEncoderSim.setDistance(Units.radiansToDegrees(SIM.getAngleRads()));
    pivotMech.setAngle(Units.radiansToDegrees(SIM.getAngleRads()));
  }

  @Override
  public void resetProfile() {
    pivotGoal = getAbsoluteEncoderPosition();
    pivotController.reset(getAbsoluteEncoderPosition());
  }
}
