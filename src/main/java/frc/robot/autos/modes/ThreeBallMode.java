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

        Pose2d startPose = new Pose2d(new Translation2d(Units.inches_to_meters(-18.5), Units.inches_to_meters(-48.2)),
                Rotation2d.fromDegrees(69));

        runAction(new ManualShootAction(3, 5));
        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAuto0Trajectory.left, false));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAuto1Trajectory.left, false));
        runAction(new WaitAction(.2));
        runAction(new CollectAction(false));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().threeBallAuto2Trajectory.left, false));
        runAction(new ManualShootAction(15, 5));
        runAction(new SetPoseAction(startPose, true));
    }
    
}
