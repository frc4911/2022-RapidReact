package frc.robot.autos.modes;

import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.SetEndOfAutoModePoseAction;
import frc.robot.actions.TwistAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;

public class HeadingTestMode extends AutoModeBase{

    @Override
    protected void routine() throws AutoModeEndedException {

         // assume heading is 0 which is set when autonomousInit is run in robot.java

        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().headingTestTrajectory.left, false));
        runAction(new TwistAction(-45.0, true, .7));
        runAction(new SetEndOfAutoModePoseAction(90.0, 0.0));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().headingTest2Trajectory.left, false));
    }
    
}
