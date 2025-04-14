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

  /** Runs the top motor of the coral intake at the specified voltage. */
  public Command runTopMotorCommand(DoubleSupplier volts) {
    return run(() -> intake.setTopMotorVoltage(volts.getAsDouble()));
  }

  /** Runs the algae intake at the specified voltage. */
  public Command runBotMotorCommand(DoubleSupplier volts) {
    return run(() -> intake.setBotMotorVoltage(volts.getAsDouble()));
  }

  /** Runs both motors at their given voltages at the same time. */
  public Command runBothMotorsCommand(DoubleSupplier voltsTop, DoubleSupplier voltsBottom) {
    return run(
        () -> {
          intake.setTopMotorVoltage(voltsTop.getAsDouble());
          intake.setBotMotorVoltage(voltsBottom.getAsDouble());
        });
  }

  /**
   * Runs both intake motors at voltages specified in {@link IntakeConstants}.
   *
   * @return Command that runs the intake until we sense a coral, then continues for a small time.
   */
  public Command intakeCoralCommand() {
    return runBothMotorsCommand(
            () -> INTAKE_CORAL_TOP_MOTOR_VOLTAGE, () -> INTAKE_CORAL_BOT_MOTOR_VOLTAGE)
        .until(getHasGamePiece())
        .andThen(waitSeconds(INTAKE_INSERTION_DELAY))
        .finallyDo(
            () -> {
              intake.setBotMotorVoltage(0);
              intake.setTopMotorVoltage(0);
            });
  }

  /**
   * Runs the top intake motor at eject voltage specified in {@link IntakeConstants}.
   *
   * @return Command that runs the intake until we no longer sense a coral, then continues for a
   *     small time.
   */
  public Command ejectCoralCommand() {
    return runTopMotorCommand(() -> EJECT_CORAL_TOP_MOTOR_VOLTAGE)
        .until(getHasGamePiece())
        .andThen(waitSeconds(EJECT_RELEASE_DELAY))
        .finallyDo(
            () -> {
              intake.setBotMotorVoltage(0);
              intake.setTopMotorVoltage(0);
            });
  }

  /**
   * MUST BE TERMINATED EXTERNALLY
   *
   * <p>Runs the BOTer intake motor at voltage specified in {@link IntakeConstants}.
   *
   * @return Command that runs the intake and then runs it at the hold voltage when interrupted.
   */
  public Command intakeAlgaeCommand() {
    return runBotMotorCommand(() -> INTAKE_ALGAE_BOT_MOTOR_VOLTAGE)
        .finallyDo(() -> intake.setBotMotorVoltage(HOLD_ALGAE_BOT_MOTOR_VOLTAGE));
  }

  /**
   * MUST BE TERMINATED EXTERNALLY
   *
   * <p>Runs the BOTer intake motor at voltage specified in {@link IntakeConstants}.
   *
   * @return Command that runs the intake and stops when interrupted
   */
  public Command ejectAlgaeCommand() {
    return runBotMotorCommand(() -> EJECT_ALGAE_BOT_MOTOR_VOLTAGE)
        .finallyDo(() -> intake.setBotMotorVoltage(0));
  }

  /**
   * Sets the voltage for both intake motors to zero.
   *
   * @return A command that will run once and terminate.
   */
  public Command stopCommand() {
    return runOnce(
        () -> {
          intake.setBotMotorVoltage(0.0);
          intake.setTopMotorVoltage(0.0);
        });
  }

  /**
   * Sets the voltage for the top intake motor to zero.
   *
   * @return A command that will run once and terminate.
   */
  public Command stopTopMotorCommand() {
    return runOnce(() -> intake.setTopMotorVoltage(0.0));
  }

  /**
   * Sets the voltage for BOTer intake motor to zero.
   *
   * @return A command that will run once and terminate.
   */
  public Command stopBotMotorCommand() {
    return runOnce(() -> intake.setBotMotorVoltage(0.0));
  }

  /**
   * @return True if there is a game piece in the intake.
   */
  public BooleanSupplier getHasGamePiece() {
    return () -> intakeInputs.hasGamePiece;
  }
}
