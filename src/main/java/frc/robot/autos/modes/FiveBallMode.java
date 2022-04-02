package frc.robot.autos.modes;

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

        runAction(new ManualShootAction(3, 5));
        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAuto0Trajectory.left, false)); //Might want to change so a three ball change wont change five ball
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAuto1Trajectory.left, false));
        runAction(new WaitAction(.2));
        runAction(new CollectAction(false)); //may want to just run collector all auto because it will only not be run during one shot as of now
        runAction(new ManualShootAction(15, 5)); //need to make shooting from distance
        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAuto0Trajectory.left, false)); //Go to human player station
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAuto0Trajectory.left, false)); //Go to shootable distance
        runAction(new CollectAction(false));
        runAction(new SetPoseAction(startPose, true));
    }
}
