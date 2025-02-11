package frc.robot.subsystems.arm;

import static frc.robot.Constants.UPDATE_PERIOD;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.currentMode;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    ElevatorIO elevator;
    PivotIO pivot;
    WristIO wrist;
    IntakeIO intake;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();
    private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

    public Arm(ElevatorIO elevatorIO,
                PivotIO pivotIO,
                WristIO wristio,
                IntakeIO intakeIO) {
        this.elevator = elevatorIO;
        this.pivot = pivotIO;
        this.wrist = wristio;
        this.intake = intakeIO;

    }

    public void periodic() {
        Logger.processInputs("Pivot", pivotInputs);
        pivot.updateInputs(pivotInputs);
        Logger.processInputs("Elevator", elevatorInputs);
        elevator.updateInputs(elevatorInputs);
        Logger.processInputs("Wrist", wristInputs);
        wrist.updateInputs(wristInputs);
        Logger.processInputs("Intake", intakeInputs);
        intake.updateInputs(intakeInputs);
    }


    /**
     * Sets the setpoint of the pivot to a certain degree.
     * @param posDeg Position in degrees to set the pivot to.
     * @return A {@link RunCommand} to set the pivot setpoint to posDeg.
     */
    public Command setPivotPositionCommand(DoubleSupplier posDeg) {
        return run(() -> pivot.setAngle(posDeg.getAsDouble()));
    }

    /**
     * Changes the setpoint of the pivot by a certain amount per second.
     * @param velocityDegPerSec Speed to move the pivot in degrees per second.
     * @return A {@link RunCommand} to change the pivot setpoint by velocityDegPerSec.
     * Resets the pivot dampening profile after completion.
     */
    public Command changePivotGoalPosition(double velocityDegPerSec) {
        return setPivotPositionCommand(() -> pivotInputs.pivotGoalPosition + (velocityDegPerSec * UPDATE_PERIOD));
    //        .finallyDo(pivot::resetProfile);
    }

    
    /**
     * Sets the setpoint of the pivot to its current position plus one degree.
     * @return A {@link RunCommand} to set the setpoint of the pivot to its current position plus once degree. 
     * This runs continously until the command ends.
     */
    public Command pivotMoveUp() {
        return new RunCommand(() -> pivot.setAngle(pivotInputs.pivotAbsolutePosition + 1.0));
    }

    /**
     * Sets the setpoint of the pivot to its current position minus one degree.
     * @return A {@link RunCommand} to set the setpoint of the pivot to its current position minus once degree. 
     * This runs continously until the command ends.
     */
    public Command pivotMoveDown() {
        return new RunCommand(() -> pivot.setAngle(pivotInputs.pivotAbsolutePosition - 1.0));
    }

    /**
     * Command to set the voltage of the elevator. Note that it will remain at this voltage until it is told otherwise.
     * @param voltage The voltage to send the elevator motors.
     * @return A runOnce command to set the voltage of the elevator to the given voltage.
     */
    public Command elevatorSetVoltage(Double voltage) {
        return this.runOnce(() -> elevator.setVoltage(voltage));
    }

    /**
     * Command that sets the elevator's voltage until interrupted.
     * @param voltage The voltage to set the elevator motors to until interrupted.
     * @return Command that sets the elevator's voltage until interrupted.
     */
    public Command elevatorMoveWithVoltage(Double voltage) {
        return this.runEnd(() -> elevator.setVoltage(voltage), () -> elevator.setVoltage(0));
    }

    /**
     * @return {@link RunCommand} that tells the wrist to move -1 degree from its current position.
     * This runs continously until the command ends.
     */
    public Command wristMoveClockwise() {
        return new RunCommand(() -> wrist.setAngle(wristInputs.wristAbsAngle - 1.0d));
    }

    /**
     * @return {@link RunCommand} that tells the wrist to move 1 degree from its current position.
     * This runs continously until the command ends.
     */
    public Command wristMoveCounterclockwise() {
        return new RunCommand(() -> wrist.setAngle(wristInputs.wristAbsAngle + 1.0d));
    }

        /**
     * Sets the setpoint of the wrist to a certain degree.
     * @param posDeg Position in degrees to set the wrist to.
     * @return A {@link RunCommand} to set the wrist setpoint to posDeg.
     */
    public Command setWristPositionCommand(DoubleSupplier posDeg) {
        return run(() -> wrist.setAngle(posDeg.getAsDouble()));
    }

    /**
     * Changes the setpoint of the wrist by a certain amount per second.
     * @param velocityDegPerSec Speed to move the wrist in degrees per second.
     * @return A {@link RunCommand} to change the wrist setpoint by velocityDegPerSec.
     * Resets the wrist dampening profile after completion.
     */
    public Command changeWristGoalPosition(double velocityDegPerSec) {
        return setWristPositionCommand(() -> wristInputs.wristGoal + (velocityDegPerSec * UPDATE_PERIOD))
            .finallyDo(wrist::resetProfile);
    }

    public Command runIntakeCommand(DoubleSupplier volts) {
        return run(() -> intake.setVoltage(volts.getAsDouble()));
    }
   /** 
    public Command zeroElevator() {
        return Commands.sequence(
            runOnce(() -> elevator.setHoming(true)),
            elevator.setVoltage(-1).
            until(elevator::elevator.atMinimum()).
            finallyDo(elevator.setVoltage(0)));
    }
    */
}
