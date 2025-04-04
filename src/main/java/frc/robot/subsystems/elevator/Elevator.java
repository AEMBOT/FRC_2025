package frc.robot.subsystems.elevator;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.constants.ElevatorConstants.ALLOWED_DEVIANCE;
import static frc.robot.constants.GeneralConstants.UPDATE_PERIOD;
import static frc.robot.constants.GeneralConstants.currentMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GeneralConstants.Mode;
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
   * Sets the setpoint of the elevator to a certain height.
   *
   * @param posIn Position in inches to set the elevator to.
   * @return A {@link RunCommand} to set the elevator setpoint to posIn.
   */
  // public Command setAngleDeg(DoubleSupplier posIn) {
  //    return run(() -> io.setAngle(posIn.getAsDouble()));
  // }

  public Command limitHeight(DoubleSupplier pivotAngle) {
    return run(() -> io.limitHeight(pivotAngle.getAsDouble()));
  }

  /**
   * Directly sets the voltage of the elevator, used for SysId.
   *
   * @param volts Voltage to apply to the elevator.
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Sets the setpoint of the elevator to a certain height in inches.
   *
   * @param posIn Position in inches to set the elevator to.
   * @return A {@link RunCommand} to set the elevator setpoint to posIn.
   */
  public Command setPosition(DoubleSupplier posMet) {
    if (currentMode == Mode.REAL) {
      return runOnce(() -> io.setHeight(posMet.getAsDouble()))
          .andThen(
              waitUntil(
                  () ->
                      Math.abs(inputs.elevatorGoalPosition - inputs.elevatorAbsolutePosition)
                          < ALLOWED_DEVIANCE));
    } else {
      return runOnce(() -> io.setHeight(posMet.getAsDouble()))
          .andThen(
              waitUntil(
                  () ->
                      Math.abs(inputs.elevatorGoalPosition - inputs.elevatorAbsolutePosition)
                          < ALLOWED_DEVIANCE * 4));
    }
  }

  /**
   * Changes the setpoint of the elevator by a certain amount per second.
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
   * @return The current position of elevator
   */
  public DoubleSupplier getPosition() {
    return () -> inputs.elevatorAbsolutePosition;
  }

  public Command zeroElevator() {
    return run(() -> io.setVoltage(-2))
        .until(() -> ((inputs.elevatorCurrentAmps[0] + inputs.elevatorCurrentAmps[1]) / 2) > 20)
        .andThen(run(() -> io.reZero()));
  }

  public void simulationPeriodic() {
    io.simulationPeriodic();
  }
}
