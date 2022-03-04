package frc.robot.autos.modes;

import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.ManualShootAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;

public class ShootOneAndDriveBackwardMode extends AutoModeBase{
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ManualShootAction(0, 3));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().backwardTrajectory.left));
    }
}
