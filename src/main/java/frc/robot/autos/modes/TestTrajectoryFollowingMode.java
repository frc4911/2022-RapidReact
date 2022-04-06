package frc.robot.autos.modes;

import java.util.ArrayList;
import java.util.List;

import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.SwerveTwistAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;
import libraries.cheesylib.autos.actions.Action;
import libraries.cheesylib.autos.actions.ParallelAction;

public class TestTrajectoryFollowingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        
        ArrayList<Action> list1 = new ArrayList<Action>();
        list1.add(new SwerveTwistAction(90));
        list1.add(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().testTrajectory.left, false));
        runAction(new SwerveTwistAction(90));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().testTrajectory.left, false));
    }
}
