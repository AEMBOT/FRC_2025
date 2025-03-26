package frc.robot.subsystems.intake;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.constants.IntakeConstants.*;

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

  /** Runs the intake at the specified voltage. */
  public Command runIntakeCommand(DoubleSupplier volts) {
    return run(() -> intake.setVoltage(volts.getAsDouble()));
  }

  /**
   * @return True if there is a game piece in the intake.
   */
  public boolean getHasGamePiece() {
    return intakeInputs.hasGamePiece;
  }

  /**
   * Runs the intake at intake voltage specified in {@link IntakeConstants}.
   *
   * @return Command that runs the intake until we sense a coral, then continues for a small time.
   */
  public Command intakeCommand() {
    return runIntakeCommand(() -> intakeVoltage)
        .until(() -> getHasGamePiece())
        .andThen(() -> waitSeconds((intakeWaitTime)))
        .finallyDo(() -> stopCommand());
  }

  /**
   * Runs the intake at eject voltage specified in {@link IntakeConstants}.
   *
   * @return Command that runs the intake until we no longer sense a coral, then continues for a
   *     small time.
   */
  public Command ejectCommand() {
    return runIntakeCommand(() -> ejectVoltage)
        .until(() -> !getHasGamePiece())
        .andThen(() -> waitSeconds((ejectWaitTime)))
        .finallyDo(() -> stopCommand());
  }

  /**
   * Sets the intake voltage to 0.
   *
   * @return A command that will run once and terminate.
   */
  public Command stopCommand() {
    return runOnce(() -> intake.setVoltage(0.0));
  }

  /**
   * public Command zeroElevator() { return Commands.sequence( runOnce(() ->
   * elevator.setHoming(true)), elevator.setVoltage(-1). until(elevator::elevator.atMinimum()).
   * finallyDo(elevator.setVoltage(0))); }
   */
}
