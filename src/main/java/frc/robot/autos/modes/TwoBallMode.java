package frc.robot.autos.modes;

import frc.robot.actions.CollectAction;
import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.ManualShootAction;
import frc.robot.actions.SetEndOfAutoModePoseAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;

public class TwoBallMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ManualShootAction(0,3));
        runAction(new CollectAction(true));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().twoBallAuto_toBallTrajectory.left));
        runAction(new CollectAction(false));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().twoBallAuto_toFenderTrajectory.left));
        runAction(new ManualShootAction(0,3));
        runAction(new SetEndOfAutoModePoseAction(-21, 0.0));
    }
}
