package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.currentMode;

public class Arm extends SubsystemBase {
    ElevatorIO elevator;
    PivotIO pivot;
    WristIO wrist;
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    public Arm() {
        switch (currentMode) {
            case REAL: {
                elevator = new ElevatorIOReal() {};
                pivot = new PivotIOReal() {};
                wrist = new WristIOReal() {};
            }
            case REPLAY: {
                elevator = new ElevatorIO() {};
                pivot = new PivotIO() {};
                wrist = new WristIO() {};
            }
            case SIM: {
                elevator = new ElevatorIO() {};
                pivot = new PivotIO() {};
                wrist = new WristIO() {};
            }
        }
    }

    public void periodic() {
        pivot.updateInputs(pivotInputs);
        elevator.updateInputs(elevatorInputs);
        wrist.updateInputs(wristInputs);
    }

    /**
     * Sets the setpoint of the pivot to its current position plus one degree.
     * @return A {@link RunCommand} to set the setpoint of the pivot to its current position plus once degree. 
     * This runs continously until the command ends.
     */
    public Command pivotMoveUp() {
        return new RunCommand(() -> pivot.setPosition(pivotInputs.pivotAbsolutePositionDeg + 1.0));
    }

    /**
     * Sets the setpoint of the pivot to its current position minus one degree.
     * @return A {@link RunCommand} to set the setpoint of the pivot to its current position minus once degree. 
     * This runs continously until the command ends.
     */
    public Command pivotMoveDown() {
        return new RunCommand(() -> pivot.setPosition(pivotInputs.pivotAbsolutePositionDeg - 1.0));
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
        return new RunCommand(() -> wrist.setSetpoint(wristInputs.position - 1.0d));
    }

    /**
     * @return {@link RunCommand} that tells the wrist to move 1 degree from its current position.
     * This runs continously until the command ends.
     */
    public Command wristMoveCounterclockwise() {
        return new RunCommand(() -> wrist.setSetpoint(wristInputs.position + 1.0d));
    }
}
