package frc.robot.autos.modes;

import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;

public class TestTrajectoryFollowingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        // runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().testTrajectory.left));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().testTrajectoryBack.left));
    }
}
