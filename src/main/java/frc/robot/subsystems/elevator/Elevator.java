package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.UPDATE_PERIOD;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public Elevator(ElevatorIO io) {
    this.io = io;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.3).per(Second),
                Volts.of(3),
                Second.of(30),
                (state) -> Logger.recordOutput(this.getName() + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));

    new Trigger(() -> inputs.openLoopStatus).onFalse(runOnce(io::resetProfile));
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

  /**
   * Applies an increasing voltage to the elevator and logs the sysId state.
   *
   * @param direction Direction to run the sysId, needs to be either kForward or kReverse
   * @return A {@link RunCommand} to run the quasistaic elevator sysId test.
   */
  public Command sysIdQuastistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command limitHeight(DoubleSupplier pivotAngle) {
    return run(() -> io.limitHeight(pivotAngle.getAsDouble()));
  }

  /**
   * Applies a constant voltage to the elevator and logs the sysId state.
   *
   * @param direction Direction to run the sysId, needs to be either kForward or kReverse
   * @return A {@link RunCommand} to run the dynamic elevator sysId test.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
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
    return run(() -> io.setHeight(posMet.getAsDouble()));
  }

  /**
   * Changes the setpoint of the elevator by a certain amount per second.
   *
   * @param velocityDegPerSec Speed to move the elevator in degrees per second.
   * @return A {@link RunCommand} to change the elevator setpoint by velocityDegPerSec. Resets the
   *     elevator dampening profile after completion.
   */
  public Command changePosition(double velocityMetPerSec) {
    return setPosition(() -> inputs.elevatorGoalPosition + (velocityMetPerSec * UPDATE_PERIOD))
        .finallyDo(io::resetProfile);
  }

  /**
   * @return The current position of elevator
   */
  public DoubleSupplier getPosition() {
    return () -> inputs.elevatorAbsolutePosition;
  }
}
