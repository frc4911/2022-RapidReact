package frc.robot.autos.modes;

import frc.robot.actions.AutoShootAction;
import frc.robot.actions.BackAction;
import frc.robot.actions.CollectAction;
import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.SetPoseAction;
import frc.robot.actions.SwerveTwistAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;
import libraries.cheesylib.autos.actions.WaitAction;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;

public class CitrusTwoBallMode extends AutoModeBase{

    @Override
    protected void routine() throws AutoModeEndedException {
                
        TrajectoryGenerator generator = TrajectoryGenerator.getInstance();
        Pose2d startPose = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));

        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(generator.getTrajectorySet().citrusTwoBallAuto0Trajectory.left, false));
        runAction(new CollectAction(false));
        runAction(new AutoShootAction(5));
        runAction(new SwerveTwistAction(90, false));
        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(generator.getTrajectorySet().citrusTwoBallAuto1Trajectory.left, false));
        runAction(new CollectAction(false));
        runAction(new SwerveTwistAction(180, true));
        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(generator.getTrajectorySet().citrusTwoBallAuto2Trajectory.left, false));
        runAction(new CollectAction(false));
        runAction(new SwerveTwistAction(120, false));
        runAction(new BackAction(true));
        runAction(new WaitAction(2));
        runAction(new BackAction(false));

        runAction(new SetPoseAction(startPose, true));
    }
    
}
