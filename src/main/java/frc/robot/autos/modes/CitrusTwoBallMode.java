package frc.robot.autos.modes;

import frc.robot.actions.AutoShootAction;
import frc.robot.actions.BackAction;
import frc.robot.actions.CollectAction;
import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.SetEndOfAutoModePoseAction;
import frc.robot.actions.SetPoseAction;
import frc.robot.actions.TwistAction;
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
        Pose2d startPose = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(-43.5)); // 316.5?

        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(generator.getTrajectorySet().citrusTwoBallAuto0Trajectory.left, false));
        runAction(new CollectAction(false));
        runAction(new AutoShootAction(5));
        runAction(new TwistAction(90, true, 1));
        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(generator.getTrajectorySet().citrusTwoBallAuto1Trajectory.left, false));
        runAction(new CollectAction(false));
        runAction(new TwistAction(-90, false, 1));
        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(generator.getTrajectorySet().citrusTwoBallAuto2Trajectory.left, false));
        runAction(new CollectAction(false));
        runAction(new TwistAction(43.5, false, 1));
        runAction(new BackAction(true));
        runAction(new WaitAction(2));
        runAction(new BackAction(false));

        // runAction(new SetPoseAction(startPose, true));
        runAction(new SetEndOfAutoModePoseAction(-43.5, 0.0));
    }
    
}
