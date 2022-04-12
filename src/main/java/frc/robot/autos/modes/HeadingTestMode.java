package frc.robot.autos.modes;

import frc.robot.actions.CollectAction;
import frc.robot.actions.DriveTrajectoryAction;
import frc.robot.actions.ManualShootAction;
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
import libraries.cheesylib.util.Units;

public class HeadingTestMode extends AutoModeBase{

    @Override
    protected void routine() throws AutoModeEndedException {

        Pose2d startPose = new Pose2d(new Translation2d(Units.inches_to_meters(-18.5), Units.inches_to_meters(-48.2)),
                Rotation2d.fromDegrees(69));

        // assume heading is 0 which is set when autonomousInit is run in robot.java

        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().headingTestTrajectory.left, false));
        runAction(new TwistAction(-45.0, true, .7));
        runAction(new SetEndOfAutoModePoseAction(90.0, 0.0));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().headingTest2Trajectory.left, false));
    }
    
}
