package frc.robot.planners;

import frc.robot.config.RobotConfiguration;
import frc.robot.config.SwerveConfiguration;
import frc.robot.constants.Constants;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.trajectory.TrajectoryIterator;
import libraries.cyberlib.kinematics.ChassisSpeeds;
import libraries.cyberlib.kinematics.SwerveDriveKinematics;
import libraries.cyberlib.kinematics.SwerveDriveOdometry;
import org.junit.jupiter.api.Test;

class DriveMotionPlannerTest {

    @Test
    void update() {
        final RobotConfiguration mRobotConfiguration = RobotConfiguration.getRobotConfiguration(Constants.kRobot2022Name);
        final SwerveConfiguration mSwerveConfiguration = mRobotConfiguration.getSwerveConfiguration();

        final SwerveDriveKinematics m_kinematics =
                new SwerveDriveKinematics(
                        mSwerveConfiguration.moduleLocations.get(0),
                        mSwerveConfiguration.moduleLocations.get(1),
                        mSwerveConfiguration.moduleLocations.get(2),
                        mSwerveConfiguration.moduleLocations.get(3));

        final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
                new Rotation2d());


        final int frequency = 50;
        final double deltaTime = 1.0 / frequency; // loop at 20 ms

        var gyro = Rotation2d.identity();  // 0 degrees is straight ahead

        TrajectoryGenerator generator = TrajectoryGenerator.getInstance();
        generator.generateTrajectories(mSwerveConfiguration.trajectoryConfig);

        var trajectory = generator.getTrajectorySet().testTrajectory.left;
        var trajectoryTime = trajectory.getLastState().t();
        System.out.format("trajectoryTime: %f\n", trajectoryTime);
        System.out.println(trajectory.toString());

        var planner = new DriveMotionPlanner();
        planner.setTrajectory(new TrajectoryIterator<>(trajectory.getIndexView()));

        var chassisSpeeds = new ChassisSpeeds();

        var position = m_odometry.getPose();
        for (double currentTime = 0; currentTime < trajectoryTime; currentTime += deltaTime) {

            var driveSignal = planner.update(currentTime, position, chassisSpeeds);

            if (driveSignal == null) {
                chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
            } else {
//                System.out.println(driveSignal.toString());
                // Convert to velocities and SI units
                var translationInput = driveSignal.getTranslation().scale(mSwerveConfiguration.maxSpeedInMetersPerSecond);
                var rotationInput = driveSignal.getRotation() * mSwerveConfiguration.maxSpeedInRadiansPerSecond;

                if (driveSignal.isFieldOriented()) {
                    // Adjust for robot heading to maintain field relative motion.
                    translationInput = translationInput.rotateBy(m_odometry.getPose().getRotation().inverse());
                }

                chassisSpeeds = new ChassisSpeeds(translationInput.x(), translationInput.y(), rotationInput);
                System.out.println("chassis Speeds " + chassisSpeeds.toString() + "; v m/s " + +translationInput.norm());
            }

            // Now calculate the new Swerve Module states using inverse kinematics.
            var swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);

            // Normalize wheels speeds if any individual speed is above the specified maximum.
            SwerveDriveKinematics.desaturateWheelSpeeds(
                    swerveModuleStates, mSwerveConfiguration.maxSpeedInMetersPerSecond);

            // Now update odometry (assume swerve modules execute perfectly )
            position = m_odometry.updateWithTime(currentTime, gyro, swerveModuleStates);

            System.out.format("time: %.2f - %s, %s\n", currentTime, position.toString(), chassisSpeeds);
        }
    }
}