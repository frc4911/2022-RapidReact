package frc.robot.paths;

import com.fasterxml.jackson.core.JsonProcessingException;
import frc.robot.config.RobotConfiguration;
import frc.robot.config.SwerveConfiguration;
import frc.robot.constants.Constants;
import libraries.cyberlib.trajectory.TrajectoryConfig;
import libraries.cyberlib.trajectory.TrajectoryUtil;
import libraries.cyberlib.trajectory.constraints.CentripetalAccelerationConstraint;
import org.junit.jupiter.api.Test;

import java.io.FileWriter;
import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class SwerveDriveTrajectoryGeneratorTest {

    @Test
    void test_generate3BallTrajectory() {
        final RobotConfiguration mRobotConfiguration = RobotConfiguration.getRobotConfiguration(Constants.kRobot2022Name);
        final SwerveConfiguration mSwerveConfiguration = mRobotConfiguration.getSwerveConfiguration();

        var generator = SwerveDriveTrajectoryGenerator.getInstance();


        var trajectoryConfig = new TrajectoryConfig(
                mSwerveConfiguration.maxSpeedInMetersPerSecond, mSwerveConfiguration.maxAccellerationInMetersPerSecondSq,
                mSwerveConfiguration.maxSpeedInRadiansPerSecond, mSwerveConfiguration.kMaxCentriptalAccelerationInMetersPerSecondSq)
                .addConstraint(new CentripetalAccelerationConstraint(mSwerveConfiguration.kMaxCentriptalAccelerationInMetersPerSecondSq));

        generator.generateTrajectories(trajectoryConfig);

        var trajectory = generator.getTrajectorySet().threeBallAutoPhase0Trajectory
                .concatenate(generator.getTrajectorySet().threeBallAutoPhase1Trajectory)
                .concatenate(generator.getTrajectorySet().threeBallAutoPhase2Trajectory);

        try (var writer = new FileWriter("c:\\Temp\\path.json", false)) {
            writer.write(TrajectoryUtil.serializeTrajectory(trajectory));
        } catch (JsonProcessingException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Test
    void test_generate5BallTrajectory() {
        final RobotConfiguration mRobotConfiguration = RobotConfiguration.getRobotConfiguration(Constants.kRobot2022Name);
        final SwerveConfiguration mSwerveConfiguration = mRobotConfiguration.getSwerveConfiguration();

        var generator = SwerveDriveTrajectoryGenerator.getInstance();


        var trajectoryConfig = new TrajectoryConfig(
                mSwerveConfiguration.maxSpeedInMetersPerSecond, mSwerveConfiguration.maxAccellerationInMetersPerSecondSq,
                mSwerveConfiguration.maxSpeedInRadiansPerSecond, mSwerveConfiguration.kMaxCentriptalAccelerationInMetersPerSecondSq)
                .addConstraint(new CentripetalAccelerationConstraint(mSwerveConfiguration.kMaxCentriptalAccelerationInMetersPerSecondSq));

        generator.generateTrajectories(trajectoryConfig);

        var trajectory = generator.getTrajectorySet().threeBallAutoPhase0Trajectory
                .concatenate(generator.getTrajectorySet().threeBallAutoPhase1Trajectory)
                .concatenate(generator.getTrajectorySet().terminalTrajectory);

        try (var writer = new FileWriter("c:\\Temp\\path.json", false)) {
            writer.write(TrajectoryUtil.serializeTrajectory(trajectory));
        } catch (JsonProcessingException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}