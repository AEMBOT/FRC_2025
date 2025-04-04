package frc.robot.subsystems.pivot;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.constants.PivotConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
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
  private boolean openLoop = true;
  private double pivotGoal;

  private final Encoder encoder = new Encoder(3, 4);
  private final ArmFeedforward pivotFF = new ArmFeedforward(0, 0.0002, 0.0722, 0);
  private final TrapezoidProfile.Constraints pivotProfile =
      new TrapezoidProfile.Constraints(100, 250);
  private final ProfiledPIDController pivotController =
      new ProfiledPIDController(1, 0, 0, pivotProfile, 0.02);
  private final EncoderSim pivotEncoderSim = new EncoderSim(encoder);
  private final MechanismRoot2d pivotRoot;

  public static final Mechanism2d pivotCanvas = new Mechanism2d(5, 10);
  public static MechanismLigament2d pivotMech;

  public PivotIOSim() {
    pivotRoot = pivotCanvas.getRoot("PivotRoot", 2.4, 0.1);
    pivotMech =
        pivotRoot.append(
            new MechanismLigament2d("PivotMech", 0.8, 0, 12, new Color8Bit(Color.kLightBlue)));
  }

  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotAbsolutePosition = Units.radiansToDegrees(SIM.getAngleRads());
    inputs.pivotAbsoluteVelocity = Units.radiansToDegrees(SIM.getVelocityRadPerSec());
    inputs.pivotAppliedVolts = SIM.getInput(0);
    inputs.pivotGoalPosition = pivotGoal;
    inputs.openLoopStatus = openLoop;
  }

  @Override
  public void setVoltage(double volts) {
    setMotorVoltage(volts);
  }

  @Override
  public void setAngle(double angle) {
    openLoop = false;
    angle = clamp(angle, MIN_ANGLE, MAX_ANGLE);
    pivotController.setGoal(angle);
    pivotGoal = angle;
  }

  private double getAbsoluteEncoderPosition() {
    return (pivotEncoderSim.getDistance());
  }

  private void setMotorVoltage(double volts) {
    openLoop = false;
    SIM.setInput(volts);
  }

  @Override
  public void simulationPeriodic() {
    double pidOutput =
        pivotController.calculate(getAbsoluteEncoderPosition())
            + pivotFF.calculate(
                pivotController.getSetpoint().position, pivotController.getSetpoint().velocity);
    setVoltage(pidOutput);

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
