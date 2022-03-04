package frc.robot.autos.modes;

import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;

public class ThreeBallMode extends AutoModeBase{
    @Override
    protected void routine() throws AutoModeEndedException {
        // start
        // shoot
        // trajectory
        // collect
        // trajectory
        // collect
        // shoot
        // done
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAutoTrajectory.left, true));
    }
}
