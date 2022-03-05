package frc.robot.autos.modes;

import java.util.Arrays;

import frc.robot.actions.CollectAction;
import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.EndTrajectoryAction;
import frc.robot.actions.ManualShootAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;
import libraries.cheesylib.autos.actions.ParallelAction;
import libraries.cheesylib.autos.actions.WaitAction;

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
    
        runAction(new ManualShootAction(2, 5));
        runAction(new CollectAction(true));
        runAction(new ParallelAction(Arrays.asList(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAuto0Trajectory.left, false),
                                    new EndTrajectoryAction(2.0))));
        runAction(new ParallelAction(Arrays.asList(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAuto1Trajectory.left, false),
                                    new EndTrajectoryAction(2.8))));
        runAction(new WaitAction(.3));
        runAction(new CollectAction(false));
        runAction(new ParallelAction(Arrays.asList(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAuto2Trajectory.left, false),
                                    new EndTrajectoryAction(2.6))));
        runAction(new ManualShootAction(5, 3));
    }
}
