package frc.robot.subsystems.pivot;

import static frc.robot.Constants.PivotConstants.*;

import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {

    private boolean openLoop = false;
    private double appliedVolts = 0.0;
    private final SingleJointedArmSim sim = pivotSim;
    private ExponentialProfile.State pivotGoal;
    private ExponentialProfile.State pivotSetpoint;
    
    public PivotIOSim() {
        pivotGoal = new ExponentialProfile.State(pivotSimGoalPosition, 0);
        pivotSetpoint = new ExponentialProfile.State(pivotSimSetpointPosition, 0);
    }

    public void updateInputs(PivotIOInputs inputs) {}

}
