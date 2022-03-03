package frc.robot.actions;

import frc.robot.subsystems.Swerve;
import libraries.cheesylib.autos.actions.RunOnceAction;

/**
 * Represents an Action to abort a running trajectory.
 */
public class OverrideTrajectory extends RunOnceAction {
    @Override
    public void runOnce() {
        Swerve.getInstance("OverrideTrajectory").overrideTrajectory(true);
    }
}
