package frc.robot.autos.modes;

import java.util.ArrayList;

import frc.robot.actions.CollectAction;
import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.ManualShootAction;
import frc.robot.actions.SetPoseAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;
import libraries.cheesylib.autos.actions.WaitAction;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.util.Units;
import libraries.cheesylib.autos.actions.Action;
import libraries.cheesylib.autos.actions.ParallelAction;


public class FiveBallMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        // start
        // shoot
        // trajectory
        // collect
        // trajectory
        // collect
        // shoot
        // trajectory
        // collect
        // trajectory
        // shoot
        // done

        Pose2d startPose = new Pose2d(new Translation2d(Units.inches_to_meters(-18.5), Units.inches_to_meters(-48.2)),
                Rotation2d.fromDegrees(69));
        ArrayList<Action> list1 = new ArrayList<Action>();
        
        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAuto0Trajectory.left, false));
        runAction(new ManualShootAction(3, 5));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAuto1Trajectory.left, false));
        runAction(new ManualShootAction(3, 5));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().fiveBallAuto2Trajectory.left, false)); //Go to human player station
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().fiveBallAuto3Trajectory.left, false)); //Go to shootable distance
        runAction(new CollectAction(false));
        runAction(new ManualShootAction(3, 5));
        runAction(new SetPoseAction(startPose, true));
    }
}
