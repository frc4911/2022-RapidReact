package frc.robot.autos.modes;

import java.util.ArrayList;

import frc.robot.actions.AutoShootAction;
import frc.robot.actions.CollectAction;
import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.SetPoseAction;
import frc.robot.actions.TwistAction;
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

        Pose2d startPose = new Pose2d(new Translation2d(Units.inches_to_meters(0), Units.inches_to_meters(0)), // -18.5, -48.2
                Rotation2d.fromDegrees(91.5));
        
        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().fiveBallAuto0Trajectory.left, false));
        runAction(new CollectAction(false));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().fiveBallAuto1Trajectory.left, false));
        runAction(new TwistAction(-45, false));
        runAction(new AutoShootAction(3));
        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().fiveBallAuto2Trajectory.left, false));
        runAction(new AutoShootAction(3));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().fiveBallAuto3Trajectory.left, false)); //Go to human player station
        runAction(new WaitAction(3));
        runAction(new CollectAction(false));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().fiveBallAuto4Trajectory.left, false)); //Go to shootable distance
        runAction(new AutoShootAction(3));
        runAction(new SetPoseAction(startPose, true));

    }
}
