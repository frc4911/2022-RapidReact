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

public class TwoBallLeftMode extends AutoModeBase{

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ManualShootAction(0, 3));
        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().twoBallAuto_toBallTrajectory.left));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().twoBallAuto_toFenderTrajectory.left));
        runAction(new CollectAction(false));
        runAction(new WaitAction(.5));
        runAction(new ManualShootAction(0, 3));
        runAction(new SetPoseAction(new Pose2d(0,0,new Rotation2d(209.0)), false));

    }
    
}
