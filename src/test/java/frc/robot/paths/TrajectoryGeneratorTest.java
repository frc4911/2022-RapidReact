package frc.robot.paths;

import frc.robot.config.RobotConfiguration;
import frc.robot.config.SwerveConfiguration;
import frc.robot.constants.Constants;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class TrajectoryGeneratorTest {

    @Test
    void getTrajectorySet() {
        final RobotConfiguration mRobotConfiguration = RobotConfiguration.getRobotConfiguration(Constants.kRobot2022Name);
        final SwerveConfiguration mSwerveConfiguration = mRobotConfiguration.getSwerveConfiguration();

        var generator = TrajectoryGenerator.getInstance();
        generator.generateTrajectories(mSwerveConfiguration.trajectoryConfig);
        var trajectorySet = generator.getTrajectorySet();
        var left = trajectorySet.testTrajectory.left;
        System.out.println(left.toString());
        System.out.println();
        System.out.println();
        var right = trajectorySet.testTrajectory.right;
        System.out.println(right.toString());
    }
}