package frc.robot.autos.modes;

import frc.robot.actions.AutoShootAction;
import frc.robot.actions.AutoShotScalerAction;
import frc.robot.actions.CollectAction;
import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.ManualShootAction;
import frc.robot.actions.SetEndOfAutoModePoseAction;
import frc.robot.actions.SetPoseAction;
import frc.robot.actions.SetShootDistanceAction;
import frc.robot.actions.SetStartPoseAction;
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

public class FourBallMode extends AutoModeBase{
    //start
        //trajectory
        //collect
        //shoot
        //trajectory
        //collect
        //trajectory
        //shoot
        //done
    @Override
    protected void routine() throws AutoModeEndedException{
        // Pose2d startPose = new Pose2d(new Translation2d(Units.inches_to_meters(-6), Units.inches_to_meters(5)), // -18.5, -48.2
        // Rotation2d.fromDegrees(10));

        // runAction(new SetStartPoseAction(startPose));

        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().fourBallAuto0Trajectory.left, false));
        runAction(new CollectAction(false));
        runAction(new AutoShootAction(30.0));
        // runAction(new CollectAction(true));
        // runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().fourBallAuto1Trajectory.left, false));
        // runAction(new WaitAction(2));
        // runAction(new CollectAction(false));
        // runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().fourBallAuto2Trajectory.left, false));
        // runAction(new AutoShootAction(30.0));
        
    }
}