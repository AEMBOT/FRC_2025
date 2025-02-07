package frc.robot.subsystems.arm;

import static frc.robot.Constants.UPDATE_PERIOD;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.ctre.phoenix6.SignalLogger;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.currentMode;

import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
    ElevatorIO elevator;
    PivotIO pivot;
    WristIO wrist;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    private final SysIdRoutine elevatorRoutine;
    private final SysIdRoutine pivotRoutine;
    private final SysIdRoutine wristRoutine;
// TODO Characterize ALL factors of interpolation for all arm systems AND ALL our PID and FF Values and
// tolerance values for BOTH Nautilus AND Dory (Nautilus first?) Also elevator position factor
// Also check routine values because the tests may not be accurate
    public Arm() {
        switch (currentMode) {
            case REAL: {
                elevator = new ElevatorIO() {};
                pivot = new PivotIOReal() {};
                wrist = new WristIOReal() {};
                break;
            }
            case REPLAY: {
                elevator = new ElevatorIO() {};
                pivot = new PivotIO() {};
                wrist = new WristIO() {};
                break;
            }
            case SIM: {
                elevator = new ElevatorIO() {};
                pivot = new PivotIO() {};
                wrist = new WristIO() {};
                break;
            }
            default: {
                break;
            }
        }

    new Trigger(() -> elevatorInputs.elevatorOpenLoopStatus).onFalse(runOnce(elevator::elevatorResetProfile));
    new Trigger(() -> pivotInputs.pivotOpenLoopStatus).onFalse(runOnce(pivot::pivotResetProfile));
    new Trigger(() -> wristInputs.wristOpenLoopStatus).onFalse(runOnce(wrist::wristResetProfile));

    elevatorRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.4).per(Second),
                Volts.of(6), //prevent burnout of motor
                Seconds.of(3), //needs atleast 3-4 seconds of data for each test
                (state) -> SignalLogger.writeString("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> elevator.setVoltage(voltage.in(Volts)), null, this));

    pivotRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(6), //prevent burnout of motor
                Seconds.of(3), //needs atleast 3-4 seconds of data for each test
                (state) -> SignalLogger.writeString("Pivot/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> pivot.setVoltage(voltage.in(Volts)), null, this));

    wristRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.4).per(Second),
                Volts.of(6), //prevent burnout of motor
                Seconds.of(5), //needs atleast 3-4 seconds of data for each test
                (state) -> Logger.recordOutput("Wrist/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> wrist.setVoltage(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        pivot.updateInputs(pivotInputs);
        elevator.updateInputs(elevatorInputs);
        wrist.updateInputs(wristInputs);
        Logger.processInputs("Pivot", pivotInputs);
        Logger.processInputs("Elevator", elevatorInputs);
        Logger.processInputs("Wrist", wristInputs);
    }


    /**
     * Sets the setpoint of the pivot to a certain degree.
     * @param posDeg Position in degrees to set the pivot to.
     * @return A {@link RunCommand} to set the pivot setpoint to posDeg.
     */
    public Command pivotSetPositionCommand(DoubleSupplier posDeg) {
        return run(() -> pivotRunPosition(posDeg.getAsDouble()));
    }

    private void pivotRunPosition(double posDeg) {
        pivot.setAngle(posDeg, elevatorInputs.elevatorPositionMet);
    }

    /**
     * Changes the setpoint of the pivot by a certain amount per second.
     * @param velocityDegPerSec Speed to move the pivot in degrees per second.
     * @return A {@link RunCommand} to change the pivot setpoint by velocityDegPerSec.
     * Resets the pivot dampening profile after completion.
     */
    public Command pivotChangeGoalPosition(double velocityDegPerSec) {
        return pivotSetPositionCommand(() -> pivotInputs.pivotGoalPosition + (velocityDegPerSec * UPDATE_PERIOD))
            .finallyDo(pivot::pivotResetProfile);
    }

    
    /**
     * Sets the setpoint of the pivot to its current position plus one degree.
     * @return A {@link RunCommand} to set the setpoint of the pivot to its current position plus once degree. 
     * This runs continously until the command ends.
     */
    public Command pivotMoveUp() {
        return run(() -> pivot.setAngle(pivotInputs.pivotAbsolutePosition + 0.01, elevatorInputs.elevatorPositionMet));
    }

    /**
     * Sets the setpoint of the pivot to its current position minus one degree.
     * @return A {@link RunCommand} to set the setpoint of the pivot to its current position minus once degree. 
     * This runs continously until the command ends.
     */
    public Command pivotMoveDown() {
        return run(() -> pivot.setAngle(pivotInputs.pivotAbsolutePosition - 0.01, elevatorInputs.elevatorPositionMet));
    }

    public Command elevatorStopCommand() {
        return runOnce (() -> elevator.setVoltage(0));
    }

    public Command pivotStopCommand() {
        return runOnce (() -> pivot.setVoltage(0));
    }

    public Command wristStopCommand() {
        return runOnce (() -> wrist.setVoltage(0));
    }
    /**
     * Command to set the voltage of the elevator. Note that it will remain at this voltage until it is told otherwise.
     * @param voltage The voltage to send the elevator motors.
     * @return A runOnce command to set the voltage of the elevator to the given voltage.
     */
    public Command elevatorSetVoltage(DoubleSupplier voltage) { 
        return this.runOnce(() -> elevator.setVoltage(voltage.getAsDouble()));
    }

    /**
     * Command that sets the elevator's voltage until interrupted.
     * @param voltage The voltage to set the elevator motors to until interrupted.
     * @return Command that sets the elevator's voltage until interrupted.
     */
    public Command elevatorMoveWithVoltage(DoubleSupplier voltage) {
        return this.runEnd(() -> elevator.setVoltage(voltage.getAsDouble()), () -> elevator.setVoltage(0));
    }

    /**
     * Command that sets the elevator's target goal using ProfiledPID and FeedForward.
     * @param goalMet Target goal (in meters) of the elevator
     * @return Command to set the elevator goal position.
     */
    public Command elevatorSetGoal(DoubleSupplier goalMet) {
        return run(() -> elevatorRunPosition(goalMet.getAsDouble()));
    }

    /**
     * Private method accessible by !!!!ONLY!!!! other commands to send elevator target goal down.
     * @param goalMet Target goal (in meters) of the elevator
     */
    private void elevatorRunPosition(double goalMet) {
        Logger.recordOutput("Elevator/goalMeters", goalMet);
        elevator.setGoalPosition(goalMet, pivotInputs.pivotAbsolutePosition);
    }

    
    public Command elevatorChangeGoal(double velocityMetPerSec) {
        return elevatorSetGoal(() -> elevatorInputs.elevatorGoalPositionMet + (velocityMetPerSec * UPDATE_PERIOD))
        .finallyDo(elevator::elevatorResetProfile);
    }

    /**
     * Command that sets the elevator's default setting.
     * @return  Command to set elevator goal to the default.
     */
    public Command elevatorGetDefault() {
        return elevatorSetGoal(() -> 0);
    }

    /**
     * Command that sets the pivot's default setting.
     * @return  Command to set pivot goal to the default.
     */
    public Command pivotGetDefault() {
        return pivotSetPositionCommand(() -> 45);
    }

    /**
     * Command that sets the wrist's default setting.
     * @return  Command to set wrist goal to the default.
     */
    public Command wristGetDefault() {
        return wristSetPositionCommand(() -> 90);
    }

    /**
     * @return {@link RunCommand} that tells the wrist to move -1 degree from its current position.
     * This runs continously until the command ends.
     */
    public Command wristMoveClockwise() {
        return run(() -> wrist.setAngle(wristInputs.wristAbsAngle - 1.0d, elevatorInputs.elevatorPositionMet, pivotInputs.pivotAbsolutePosition));
    }

    /**
     * @return {@link RunCommand} that tells the wrist to move 1 degree from its current position.
     * This runs continously until the command ends.
     */
    public Command wristMoveCounterclockwise() {
        return run(() -> wrist.setAngle(wristInputs.wristAbsAngle + 1.0d, elevatorInputs.elevatorPositionMet, pivotInputs.pivotAbsolutePosition));
    }

        /**
     * Sets the setpoint of the wrist to a certain degree.
     * @param posDeg Position in degrees to set the wrist to.
     * @return A {@link RunCommand} to set the wrist setpoint to posDeg.
     */
    public Command wristSetPositionCommand(DoubleSupplier posDeg) {
        return run(() -> wristRunPositionCommand(posDeg.getAsDouble()));
    }

    private void wristRunPositionCommand(double posDeg) {
        wrist.setAngle(posDeg, elevatorInputs.elevatorPositionMet, pivotInputs.pivotAbsolutePosition);
    }

    /**
     * Changes the setpoint of the wrist by a certain amount per second.
     * @param velocityDegPerSec Speed to move the wrist in degrees per second.
     * @return A {@link RunCommand} to change the wrist setpoint by velocityDegPerSec.
     * Resets the wrist dampening profile after completion.
     */
    public Command wristChangeGoalPosition(double velocityDegPerSec) {
        return wristSetPositionCommand(() -> wristInputs.wristGoal + (velocityDegPerSec * UPDATE_PERIOD))
            .finallyDo(wrist::wristResetProfile);
    }

    public Command runElevatorCharacterization() {
    return Commands.sequence(
        elevatorRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        elevatorStopCommand(),
        Commands.waitSeconds(2.0),
        elevatorRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        elevatorStopCommand(),
        Commands.waitSeconds(2.0),
        elevatorRoutine.dynamic(SysIdRoutine.Direction.kForward),
        elevatorStopCommand(),
        Commands.waitSeconds(2.0),
        elevatorRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }

    public Command runPivotCharacterization() {
    return Commands.sequence(
        pivotRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        pivotStopCommand(),
        Commands.waitSeconds(2.0),
        pivotRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        pivotStopCommand(),
        Commands.waitSeconds(2.0),
        pivotRoutine.dynamic(SysIdRoutine.Direction.kForward),
        pivotStopCommand(),
        Commands.waitSeconds(2.0),
        pivotRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command runWristCharacterization() {
    return Commands.sequence(
        wristRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        wristStopCommand(),
        Commands.waitSeconds(2.0),
        wristRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        wristStopCommand(),
        Commands.waitSeconds(2.0),
        wristRoutine.dynamic(SysIdRoutine.Direction.kForward),
        wristStopCommand(),
        Commands.waitSeconds(2.0),
        wristRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }
}
