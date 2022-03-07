package frc.robot.autos.modes;

import frc.robot.actions.CollectAction;
import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.ManualShootAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;
import libraries.cheesylib.autos.actions.WaitAction;

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
    }
    
}
