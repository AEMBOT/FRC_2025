package frc.robot.subsystems.intake;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import java.util.function.BooleanSupplier;
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

  /** Runs the coral intake at the specified voltage. */
  public Command runTopMotorCommand(DoubleSupplier volts) {
    return run(() -> intake.setTopMotorVoltage(volts.getAsDouble()));
  }

  /** Runs the algae intake at the specified voltage. */
  public Command runLowMotorCommand(DoubleSupplier volts) {
    return run(() -> intake.setLowMotorVoltage(volts.getAsDouble()));
  }

  /**
   * @return True if there is a game piece in the intake.
   */
  public BooleanSupplier getHasGamePiece() {
    return () -> intakeInputs.hasGamePiece;
  }

  /**
   * Runs both intake motors at voltages specified in {@link IntakeConstants}.
   *
   * @return Command that runs the intake until we sense a coral, then continues for a small time.
   */
  public Command intakeCoralCommand() {
    return runTopMotorCommand(() -> intakeCoralTopMotorVoltage)
        .alongWith(runLowMotorCommand(() -> intakeCoralLowMotorVoltage))
        .until(getHasGamePiece())
        .andThen(waitSeconds(intakeWaitTime))
        .finallyDo(() -> stopCommand());
  }

  /**
   * Runs the top intake motor at eject voltage specified in {@link IntakeConstants}.
   *
   * @return Command that runs the intake until we no longer sense a coral, then continues for a
   *     small time.
   */
  public Command ejectCoralCommand() {
    return runTopMotorCommand(() -> ejectCoralTopMotorVoltage)
        .until(getHasGamePiece())
        .andThen(waitSeconds(ejectWaitTime))
        .finallyDo(() -> stopTopMotorCommand());
  }

  /**
   * MUST BE TERMINATED EXTERNALLY <p>
   * 
   * Runs the lower intake motor at voltage specified in {@link IntakeConstants}.
   *
   * @return Command that runs the intake and then runs it at the hold voltage when interrupted.
   */
  public Command intakeAlgaeCommand() {
    return runLowMotorCommand(() -> intakeAlgaeLowMotorVoltage)
        .finallyDo(() -> runLowMotorCommand(() -> holdAlgaeLowMotorVoltage));
  }

    /**
   * MUST BE TERMINATED EXTERNALLY <p>
   * 
   * Runs the lower intake motor at voltage specified in {@link IntakeConstants}.
   *
   * @return Command that runs the intake and stops when interrupted
   */
  public Command ejectAlgaeCommand() {
    return runLowMotorCommand(() -> ejectAlgaeLowMotorVoltage)
        .finallyDo(() -> stopCommand());
  }


  /**
   * Sets the voltage for both intake motors to zero.
   *
   * @return A command that will run once and terminate.
   */
  public Command stopCommand() {
    return runOnce(() -> intake.setLowMotorVoltage(0.0))
        .alongWith(runOnce(() -> intake.setTopMotorVoltage(0.0)));
  }

  /**
   * Sets the voltage for the top intake motor to zero.
   *
   * @return A command that will run once and terminate.
   */
  public Command stopTopMotorCommand() {
    return 
        runOnce(() -> intake.setTopMotorVoltage(0.0));
  }

  /**
   * Sets the voltage for lower intake motor to zero.
   *
   * @return A command that will run once and terminate.
   */
  public Command stopLowMotorCommand() {
    return 
        runOnce(() -> intake.setLowMotorVoltage(0.0));
  }

  /**
   * public Command zeroElevator() { return Commands.sequence( runOnce(() ->
   * elevator.setHoming(true)), elevator.setVoltage(-1). until(elevator::elevator.atMinimum()).
   * finallyDo(elevator.setVoltage(0))); }
   */
}
