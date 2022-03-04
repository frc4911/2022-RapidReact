package libraries.cyberlib.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfiguration;
import frc.robot.config.SwerveConfiguration;
import frc.robot.constants.Constants;
import libraries.cyberlib.trajectory.constraints.TrajectoryConstraint;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static edu.wpi.first.math.util.Units.feetToMeters;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static org.junit.jupiter.api.Assertions.*;

class TrajectoryGeneratorTest {
    static Trajectory getTrajectory(List<? extends TrajectoryConstraint> constraints) {
        final double maxVelocity = feetToMeters(12.0);
        final double maxAccel = feetToMeters(12);

        final double kRobotWidth = inchesToMeters(30);
        final double kRobotLength = inchesToMeters(30);
        final double kRadius = Math.hypot(kRobotWidth / 2, kRobotLength / 2);

        final double kAngularVelocity = Math.min(maxVelocity / kRadius, Math.toRadians(270));  // radians/s
        final double kAngularAcceleration = kAngularVelocity / 0.4;

        // 2018 cross scale auto waypoints.
        var sideStart = new Pose2d(feetToMeters(1.54), feetToMeters(23.23),
                Rotation2d.fromDegrees(-180));
        var crossScale = new Pose2d(feetToMeters(23.7), feetToMeters(6.8),
                Rotation2d.fromDegrees(-160));

        var waypoints = new ArrayList<Pose2d>();
        waypoints.add(sideStart);
        waypoints.add(sideStart.plus(
                new Transform2d(new Translation2d(feetToMeters(-13), feetToMeters(0)),
                        new Rotation2d())));
        waypoints.add(sideStart.plus(
                new Transform2d(new Translation2d(feetToMeters(-19.5), feetToMeters(5)),
                        Rotation2d.fromDegrees(-90))));
        waypoints.add(crossScale);

        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAccel, kAngularVelocity, kAngularAcceleration)
                .setReversed(true)
                .addConstraints(constraints);

        return TrajectoryGenerator.generateTrajectory(waypoints, config);
    }

    @Test
    @SuppressWarnings("LocalVariableName")
    void testGenerationAndConstraints() {
        // TODO:  INVESTIGATE WHY THIS FAILS
        Trajectory trajectory = getTrajectory(new ArrayList<>());

        double duration = trajectory.getTotalTimeSeconds();
        double t = 0.0;
        double dt = 0.02;

        while (t < duration) {
            var point = trajectory.sample(t);
            System.out.println(point.toString());
            t += dt;
            assertAll(
                    () -> assertTrue(Math.abs(point.velocityMetersPerSecond) < feetToMeters(12.0) + 0.05),
                    () -> assertTrue(Math.abs(point.accelerationMetersPerSecondSq) < feetToMeters(12.0)
                            + 0.05)
            );
        }
    }

    @Test
    void testMalformedTrajectory() {
        var traj =
                TrajectoryGenerator.generateTrajectory(
                        Arrays.asList(
                                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                                new Pose2d(1, 0, Rotation2d.fromDegrees(180))
                        ),
                        new TrajectoryConfig(feetToMeters(12), feetToMeters(12))
                );

        assertEquals(traj.getStates().size(), 1);
        assertEquals(traj.getTotalTimeSeconds(), 0);
    }

    @Test
    void testSwervePath() {
        final RobotConfiguration mRobotConfiguration = RobotConfiguration.getRobotConfiguration(Constants.kRobot2022Name);
        final SwerveConfiguration mSwerveConfiguration = mRobotConfiguration.getSwerveConfiguration();

        List<Pose2d> wayPoints = new ArrayList<>();
        wayPoints.add(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
        wayPoints.add(new Pose2d(new Translation2d(Units.inchesToMeters(50), Units.inchesToMeters(50)), Rotation2d.fromDegrees(0)));
        wayPoints.add(new Pose2d(new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(10)), Rotation2d.fromDegrees(0)));
        wayPoints.add(new Pose2d(new Translation2d(Units.inchesToMeters(120), 0), Rotation2d.fromDegrees(0)));
        wayPoints.add(new Pose2d(new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(-10)), Rotation2d.fromDegrees(0)));

        TrajectoryConfig config = new TrajectoryConfig(
                mSwerveConfiguration.trajectoryConfig.getMaxVelocity(),
                mSwerveConfiguration.trajectoryConfig.getMaxAcceleration(),
                mSwerveConfiguration.trajectoryConfig.getMaxAngularVelocity(),
                mSwerveConfiguration.trajectoryConfig.getMaxAngularAcceleration())
                .setReversed(mSwerveConfiguration.trajectoryConfig.isReversed())
                .addConstraints(new ArrayList<>());

        var trajectory = TrajectoryGenerator.generateTrajectory(wayPoints, config);
        System.out.println("Trajectory");
        System.out.println(trajectory.toString());
    }

    @Test
    void testStraightSwervePath() {
        final RobotConfiguration mRobotConfiguration = RobotConfiguration.getRobotConfiguration(Constants.kRobot2022Name);
        final SwerveConfiguration mSwerveConfiguration = mRobotConfiguration.getSwerveConfiguration();

        List<Pose2d> wayPoints = new ArrayList<>();
        wayPoints.add(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
        wayPoints.add(new Pose2d(new Translation2d(Units.inchesToMeters(50), Units.inchesToMeters(50)), Rotation2d.fromDegrees(0)));

        TrajectoryConfig config = new TrajectoryConfig(
                mSwerveConfiguration.trajectoryConfig.getMaxVelocity(),
                mSwerveConfiguration.trajectoryConfig.getMaxAcceleration(),
                mSwerveConfiguration.trajectoryConfig.getMaxAngularVelocity(),
                mSwerveConfiguration.trajectoryConfig.getMaxAngularAcceleration())
                .setReversed(mSwerveConfiguration.trajectoryConfig.isReversed())
                .addConstraints(new ArrayList<>());

        var trajectory = TrajectoryGenerator.generateTrajectory(wayPoints, config);
        System.out.println("Trajectory");
        System.out.println(trajectory.toString());
    }

    @Test
    void testNinteyTurnSwervePath() {
        final RobotConfiguration mRobotConfiguration = RobotConfiguration.getRobotConfiguration(Constants.kRobot2022Name);
        final SwerveConfiguration mSwerveConfiguration = mRobotConfiguration.getSwerveConfiguration();

        List<Pose2d> wayPoints = new ArrayList<>();
        wayPoints.add(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
        wayPoints.add(new Pose2d(new Translation2d(Units.inchesToMeters(50), Units.inchesToMeters(0)), Rotation2d.fromDegrees(0)));

        TrajectoryConfig config = new TrajectoryConfig(
                mSwerveConfiguration.trajectoryConfig.getMaxVelocity(),
//                Units.feetToMeters(14.2),
                mSwerveConfiguration.trajectoryConfig.getMaxAcceleration(),
                mSwerveConfiguration.trajectoryConfig.getMaxAngularVelocity(),
                mSwerveConfiguration.trajectoryConfig.getMaxAngularAcceleration())
                .setReversed(mSwerveConfiguration.trajectoryConfig.isReversed())
                .addConstraints(new ArrayList<>());

        var trajectory = TrajectoryGenerator.generateTrajectory(wayPoints, config);
        System.out.println("Trajectory");
        System.out.println(trajectory.toString());
    }

}
