package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  IntakeIO intake;
  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {
    this.intake = intakeIO;
  }

  public void periodic() {
    Logger.processInputs("Intake", intakeInputs);
    intake.updateInputs(intakeInputs);
  }

  public Command runIntakeCommand(DoubleSupplier volts) {
    return run(() -> intake.setVoltage(volts.getAsDouble()));
  }

  public DoubleSupplier getGamePiecePosition() {
    return () -> intakeInputs.gamePieceLocation;
  }
  /**
   * public Command zeroElevator() { return Commands.sequence( runOnce(() ->
   * elevator.setHoming(true)), elevator.setVoltage(-1). until(elevator::elevator.atMinimum()).
   * finallyDo(elevator.setVoltage(0))); }
   */
}
