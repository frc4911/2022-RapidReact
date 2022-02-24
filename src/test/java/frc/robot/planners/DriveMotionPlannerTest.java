package frc.robot.planners;

import frc.robot.config.Robot2022;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.trajectory.TrajectoryIterator;
import libraries.cyberlib.control.HolonomicFeedforward;
import libraries.cyberlib.control.HolonomicTrajectoryFollower;
import libraries.cyberlib.control.PidGains;
import libraries.cyberlib.control.SwerveDriveFeedforwardGains;
import libraries.cyberlib.kinematics.ChassisSpeeds;
import libraries.cyberlib.kinematics.SwerveDriveKinematics;
import libraries.cyberlib.kinematics.SwerveDriveOdometry;
import libraries.cyberlib.utils.HolonomicDriveSignal;
import org.junit.jupiter.api.Test;

import java.util.Optional;

import static org.junit.jupiter.api.Assertions.*;

class DriveMotionPlannerTest {

    private final Translation2d m_fl = new Translation2d(12, 12);
    private final Translation2d m_fr = new Translation2d(12, -12);
    private final Translation2d m_bl = new Translation2d(-12, 12);
    private final Translation2d m_br = new Translation2d(-12, -12);

    private final SwerveDriveKinematics m_kinematics =
            new SwerveDriveKinematics(m_fl, m_fr, m_bl, m_br);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
            new Rotation2d());


    @Test
    void update() {
        final int frequency = 50;
        final double deltaTime = 1.0 / frequency; // loop at 20 ms

        var gyro = Rotation2d.identity();  // 0 degrees is straight ahead

        TrajectoryGenerator generator = TrajectoryGenerator.getInstance();
        generator.generateTrajectories();

        var trajectory = generator.getTrajectorySet().testTrajectory.left;
        var trajectoryTime = trajectory.getLastState().t();
        System.out.println(String.format("trajectoryTime: %f", trajectoryTime));
        System.out.println(trajectory.toString());

        var mRobotConfiguration = new Robot2022();
        var mServeConfiguration = mRobotConfiguration.getSwerveConfiguration();

        var planner = new DriveMotionPlanner();
        planner.setTrajectory(new TrajectoryIterator<>(trajectory.getIndexView()));

        var chassisSpeeds = new ChassisSpeeds();

        var position = m_odometry.getPose();
        for (double currentTime = 0; currentTime < trajectoryTime; currentTime += deltaTime) {

             var driveSignal = planner.update(currentTime, position, chassisSpeeds);

            if (driveSignal == null) {
                chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
            } else {
                System.out.println(driveSignal.toString());
                // Convert to velocities and SI units
                var translationInput = driveSignal.getTranslation().scale(mServeConfiguration.maxSpeedInMetersPerSecond);
                var rotationInput = driveSignal.getRotation() * mServeConfiguration.maxSpeedInRadiansPerSecond;

                if (driveSignal.isFieldOriented()) {
                    // Adjust for robot heading to maintain field relative motion.
                    translationInput = translationInput.rotateBy(m_odometry.getPose().getRotation().inverse());
                }

                chassisSpeeds = new ChassisSpeeds(translationInput.x(), translationInput.y(), rotationInput);
            }

            // Now calculate the new Swerve Module states using inverse kinematics.
            var swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);

            // Normalize wheels speeds if any individual speed is above the specified maximum.
            SwerveDriveKinematics.desaturateWheelSpeeds(
                    swerveModuleStates, mServeConfiguration.maxSpeedInMetersPerSecond);

            // Now update odometry (assume swerve modules execute perfectly )
            position = m_odometry.updateWithTime(currentTime, gyro, swerveModuleStates);
            System.out.printf("time: %.2f - %s%n", currentTime, position.toString());
        }
    }
}