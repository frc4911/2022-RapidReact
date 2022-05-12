package frc.robot.autos.modes;

import frc.robot.actions.AutoShootAction;
import frc.robot.actions.BackAction;
import frc.robot.actions.CollectAction;
import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.SetEndOfAutoModePoseAction;
import frc.robot.actions.TwistAction;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeEndedException;
import libraries.cheesylib.autos.actions.WaitAction;

public class CitrusTwoBallMode extends AutoModeBase{

    @Override
    protected void routine() throws AutoModeEndedException {
                
        TrajectoryGenerator generator = TrajectoryGenerator.getInstance();

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

        runAction(new SetEndOfAutoModePoseAction(-43.5, 0.0));
    }
    
}
