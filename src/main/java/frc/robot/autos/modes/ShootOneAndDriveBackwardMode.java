package frc.robot.autos.modes;

import frc.robot.actions.AutoShootAction;
import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.SetEndOfAutoModePoseAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;

public class ShootOneAndDriveBackwardMode extends AutoModeBase{
    @Override
    protected void routine() throws AutoModeEndedException {
        TrajectoryGenerator generator = TrajectoryGenerator.getInstance();

        runAction(new DriveTrajectoryAction(generator.getTrajectorySet().citrusTwoBallAuto0Trajectory.left, false));
        runAction(new AutoShootAction(5));

        runAction(new SetEndOfAutoModePoseAction(-43.5, 0.0));
    }
}
