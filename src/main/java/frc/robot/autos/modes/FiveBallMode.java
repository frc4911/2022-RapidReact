package frc.robot.autos.modes;

import java.util.ArrayList;

import frc.robot.actions.AutoShootAction;
import frc.robot.actions.AutoShotScalerAction;
import frc.robot.actions.CollectAction;
import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.ManualShootAction;
import frc.robot.actions.SetEndOfAutoModePoseAction;
import frc.robot.actions.SetPoseAction;
import frc.robot.actions.SetShootDistanceAction;
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

        // Pose2d startPose = new Pose2d(new Translation2d(Units.inches_to_meters(0), Units.inches_to_meters(0)), // -18.5, -48.2
        //         Rotation2d.fromDegrees(91.5));
        // phase 0
        //    inform shooter of coming shot
        runAction(new AutoShotScalerAction(3.5));
        runAction(new SetShootDistanceAction(72.0));
        // deploy collector
        runAction(new CollectAction(true));
        // back up just enough to grab second ball
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().fiveBallAuto0Trajectory.left, false));
        runAction(new CollectAction(false));
        // shoot, this will retract the collector
        runAction(new AutoShootAction(30.0));
        // twist to align with picking up the next ball
        runAction(new TwistAction(-45, false));
        // deploy collector
        runAction(new CollectAction(true));
        // prepare for shot
        runAction(new SetShootDistanceAction(86.0));
        // move to next ball (this will also collect)
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().fiveBallAuto1Trajectory.left, false));
        // shoot this will change the alignment a little
        runAction(new CollectAction(false));
        runAction(new AutoShootAction(30.0));
        // align for picking up balls at feeder station, retracts collector
        runAction(new TwistAction(-60, false));
        runAction(new CollectAction(true));
        // drive to feeder station
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().fiveBallAuto2Trajectory.left, false));
        // first ball is collected automatically, second ball is rolled in by human player who has .75 seconds
        runAction(new WaitAction(.5));
        // drive back to second shot position
        runAction(new CollectAction(false));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().fiveBallAuto3Trajectory.left, false)); //Go to human player station
        runAction(new TwistAction(-30, false));
        runAction(new AutoShootAction(30.0));
        runAction(new SetEndOfAutoModePoseAction(90.0, 0.0));
        runAction(new AutoShotScalerAction(Double.NaN));  // NaN returns the scaler to default
        // runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().fiveBallAuto4Trajectory.left, false));

    }
}
