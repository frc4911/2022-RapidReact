package frc.robot.autos.modes;

import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.TwistAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;

public class TestTrajectoryFollowingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        
        runAction(new TwistAction(90, true));
        runAction(new TwistAction(90, false));

    }
}
