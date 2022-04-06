package frc.robot.autos.modes;

import frc.robot.actions.AutoShootAction;
import frc.robot.actions.CollectAction;
import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.SetPoseAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;

public class BackOfTarmacTwoBallMode extends AutoModeBase{

    @Override
    protected void routine() throws AutoModeEndedException {
                
        TrajectoryGenerator generator = TrajectoryGenerator.getInstance();
        Pose2d startPose = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));

        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(generator.getTrajectorySet().citrusTwoBallAuto0Trajectory.left, false));
        runAction(new CollectAction(false));
        runAction(new AutoShootAction(5));

        runAction(new SetPoseAction(startPose, true));
    }
    
}
