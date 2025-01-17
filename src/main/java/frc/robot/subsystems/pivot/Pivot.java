package frc.robot.subsystems.pivot;

import static frc.robot.Constants.PivotConstants.pivotSysIdRampRate;
import static frc.robot.Constants.PivotConstants.pivotSysIdStepVolt;
import static frc.robot.Constants.PivotConstants.pivotSysIdTimeout;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.PivotConstants.*;
import static frc.robot.Constants.UPDATE_PERIOD;
import static java.lang.Math.abs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
    
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public Pivot(PivotIO io) {
        this.io = io;

        new Trigger(() -> inputs.openLoopStatus).onFalse(runOnce(io::resetProfile));

        sysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    Volts.of(pivotSysIdRampRate).per(Seconds),
                    Volts.of(pivotSysIdStepVolt),
                    Seconds.of(pivotSysIdTimeout),
                (state) -> Logger.recordOutput("Pivot?SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        Logger.recordOutput(
            "Pivot/Running Command", 
            Optional.ofNullable(this.getCurrentCommand()).map(Command::getName).orElse("None"));
    }

    /**  */
    @AutoLogOutput
    public boolean atGoal() {
        return abs(inputs.pivotAbsolutePosition - inputs.pivotGoalPosition) < pivotAngleAllowedDeviance;
    }

    /**  */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /**  */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /** @return The pose of the pivot from the center of the robot */
    public Pose3d getPose() {
        return new Pose3d(pivotTranslationFromRobot, new Rotation3d(0, -Units.degreesToRadians(inputs.pivotAbsolutePosition), 0));
    }
    
    /** @return The current angle of the pivot, in degrees */
    public double getAngle() {
        return inputs.pivotAbsolutePosition;
    }

    /**  */
    public Command runVoltsCommand(double volts) {
        return run(() -> runVolts(volts)).finallyDo(() -> runVolts(0));
    }

    /**  */
    public Command changeGoalPosition(double velocity) {
        return setPositionCommand(() -> inputs.pivotGoalPosition + (velocity * UPDATE_PERIOD))
            .finallyDo(io::resetProfile);
    }

    /**  */
    public Command setPositionCommand(DoubleSupplier angle) {
        return run(() -> runPosition(angle.getAsDouble()));
    }

    /** */
    public Command getDefault() {
        return setPositionCommand(() -> pivotDefaultAngle);
    }

    /**  */
    private void runVolts(double volts) {
        io.setVoltage(volts);
    }

    /** */
    private void runPosition(double angle) {
        Logger.recordOutput("Pivot/GoalAngle", angle);
        io.setAngle(angle);
    }

}
