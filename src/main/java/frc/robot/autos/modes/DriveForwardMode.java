package frc.robot.autos.modes;

import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;

public class DriveForwardMode extends AutoModeBase{
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().forwardTrajectory.left));        
    }
}
