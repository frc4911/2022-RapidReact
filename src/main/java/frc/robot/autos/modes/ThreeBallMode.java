package frc.robot.autos.modes;

import java.util.Arrays;

import frc.robot.actions.CollectAction;
import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.EndTrajectoryAction;
import frc.robot.actions.ManualShootAction;
import frc.robot.actions.SetPoseAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;
import libraries.cheesylib.autos.actions.ParallelAction;
import libraries.cheesylib.autos.actions.WaitAction;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;

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
    
        runAction(new ManualShootAction(3, 5));
        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAuto0Trajectory.left, false));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAuto1Trajectory.left, false));
        runAction(new WaitAction(.2));
        runAction(new CollectAction(false));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAuto2Trajectory.left, false));
        runAction(new ManualShootAction(15, 5));
        runAction(new SetPoseAction(new Pose2d(0,0,new Rotation2d(119.0))));

    }
}
