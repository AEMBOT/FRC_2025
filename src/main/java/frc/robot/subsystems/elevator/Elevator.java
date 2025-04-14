package frc.robot.subsystems.elevator;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.constants.ElevatorConstants.*;
import static frc.robot.constants.GeneralConstants.UPDATE_PERIOD;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    Logger.processInputs(this.getName(), inputs);
    io.updateInputs(inputs);
  }

  /**
   * Directly sets the voltage of the elevator. Don't use this without reason.
   *
   * @param volts Voltage to apply to the elevator.
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /** Sets our elevator voltage to 0. */
  public void stopElevator() {
    io.setVoltage(0);
  }

  /**
   * Sets the setpoint of the elevator to a certain height in inches.
   *
   * @param posIn Position in inches to set the elevator to.
   * @return A {@link RunCommand} to set the elevator setpoint to posIn.
   */
  public Command setPosition(DoubleSupplier posMet) {
    return runOnce(() -> io.setHeight(posMet.getAsDouble()))
        .andThen(
            waitUntil(
                () ->
                    Math.abs(inputs.elevatorGoalPosition - inputs.elevatorAbsolutePosition)
                        < ALLOWED_DEVIANCE));
  }

  /**
   * Changes the setpoint of the elevator by the given meters per second.
   *
   * @param velocityDegPerSec Speed to move the elevator in degrees per second.
   * @return A {@link RunCommand} to change the elevator setpoint by velocityDegPerSec. Resets the
   *     elevator dampening profile after completion.
   */
  public Command changePosition(double velocityMetPerSec) {
    return run(() ->
            io.setHeight(inputs.elevatorGoalPosition + (velocityMetPerSec * UPDATE_PERIOD)))
        .finallyDo(() -> io.resetProfile());
  }

  /**
   * Limits our elevator height based on our pivot's angle.
   *
   * @param pivotAngle
   * @return Our maximum height limit, given our pivot angle.
   */
  public Command limitHeight(DoubleSupplier pivotAngle) {
    return run(() -> io.limitHeight(pivotAngle.getAsDouble()));
  }

  /**
   * Directly gives the elevator voltage until we detect hitting a hardstop by measuring our amps.
   * Then, set the position as the new motor zero and then stop the elevator.
   */
  public Command zeroElevator() {
    DoubleSupplier avgAmps =
        () -> (inputs.elevatorCurrentAmps[0] + inputs.elevatorCurrentAmps[1]) / 2;

    Logger.recordOutput("ElevatorAverageAmps", avgAmps.getAsDouble());

    return run(() -> setVoltage(ZEROING_VOLTAGE))
        .until(() -> avgAmps.getAsDouble() > ELEVATOR_ZEROING_MAX_AMPS)
        .andThen(runOnce(() -> io.setMotorZero()))
        .andThen(runOnce(() -> stopElevator()));
  }

  /**
   * @return The current position of elevator.
   */
  public DoubleSupplier getPosition() {
    return () -> inputs.elevatorAbsolutePosition;
  }
}
