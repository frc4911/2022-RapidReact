package frc.robot.autos.modes;

import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.ManualShootAction;
import frc.robot.actions.SetPoseAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;

public class ShootOneAndDriveBackwardMode extends AutoModeBase{
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ManualShootAction(0, 3));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().backwardTrajectory.left));
        //runAction(new SetPoseAction(new Pose2d(0,0,new Rotation2d(29.0))));
    }
}
