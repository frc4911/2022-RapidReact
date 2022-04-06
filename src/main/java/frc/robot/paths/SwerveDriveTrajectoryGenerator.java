package frc.robot.paths;

import edu.wpi.first.wpilibj.Timer;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Pose2dWithCurvature;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.trajectory.timing.TimedState;
import libraries.cheesylib.util.Units;
import libraries.cyberlib.paths.Path;
import libraries.cyberlib.paths.PathBuilder;
import libraries.cyberlib.paths.SplinePath;
import libraries.cyberlib.paths.SplinePathBuilder;
import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;
import libraries.cyberlib.spline.QuinticHermiteSpline3D;
import libraries.cyberlib.trajectory.Trajectory;
import libraries.cyberlib.trajectory.TrajectoryConfig;
import libraries.cyberlib.trajectory.TrajectoryGenerator;
import libraries.cyberlib.trajectory.TrajectoryUtil;

import java.util.ArrayList;
import java.util.List;

import static libraries.cyberlib.trajectory.swerve.SwerveTrajectoryGenerator.create_trajectory;

public class SwerveDriveTrajectoryGenerator {
    private static SwerveDriveTrajectoryGenerator mInstance;
    private SwerveDriveTrajectoryGenerator.TrajectorySet mTrajectorySet = null;

    private SwerveDriveTrajectoryGenerator() {
    }

    public static SwerveDriveTrajectoryGenerator getInstance() {
        if (mInstance == null) {
            mInstance = new SwerveDriveTrajectoryGenerator();
        }
        return mInstance;
    }

    public static Trajectory generateTrajectory(final SplinePath path, TrajectoryConfig config) {
        List<QuinticHermiteSpline3D> splines = new ArrayList<>();

        var trajectory = create_trajectory(
                path.waypoints,
                path.startHeading.getRadians(),
                path.endHeading.getRadians(),
                config,
                splines
        );

        return trajectory;
    }

    public static Trajectory generateTrajectory(final Path path, TrajectoryConfig config) {
        var points = path.parameterize();
        var trajectory = TrajectoryGenerator.generateTrajectoryFromPoints(points, config);
        return trajectory;
    }

    public void generateTrajectories(TrajectoryConfig config) {
        if (mTrajectorySet == null) {
            double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories...");
            mTrajectorySet = new SwerveDriveTrajectoryGenerator.TrajectorySet(config);
            System.out.println(
                    "Finished trajectory generation in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }

    public SwerveDriveTrajectoryGenerator.TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public class TrajectorySet {

        public final Trajectory testTrajectory;
        public final Trajectory testTrajectoryBack;
        public final Trajectory threeBallAutoPhase0Trajectory;
        public final Trajectory threeBallAutoPhase1Trajectory;
        public final Trajectory threeBallAutoPhase2Trajectory;
        public final Trajectory terminalTrajectory;

        private TrajectorySet(TrajectoryConfig config) {
            // TODO: Implement deep clone so a trajectory generator function can freely modify the configuration.
            // NOTE: Constraints are not deep copied for now.
            threeBallAutoPhase0Trajectory = getThreeBallAutoPhase0Trajectory(config);
            threeBallAutoPhase1Trajectory = getThreeBallAutoPhase1Trajectory(config);
            threeBallAutoPhase2Trajectory = getThreeBallAutoPhase2Trajectory(config);
            terminalTrajectory = getTerminalTrajectory(config);

            testTrajectory = getTestTrajectory(config);
            testTrajectoryBack = getTestTrajectory(config);
        }

        private Trajectory getThreeBallAutoPhase0Trajectory(TrajectoryConfig config) {
            var builder = new PathBuilder(
                    new edu.wpi.first.math.geometry.Translation2d(Units.inches_to_meters(-18.5), Units.inches_to_meters(-48.2)),
                    edu.wpi.first.math.geometry.Rotation2d.fromDegrees(69))
                    .lineTo(new edu.wpi.first.math.geometry.Translation2d(Units.inches_to_meters(-22.2), Units.inches_to_meters(-128.9)),
                            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90));

//            var builder = new SplinePathBuilder(
//                    new Translation2d(Units.inches_to_meters(-18.5), Units.inches_to_meters(-48.2)),
//                    Rotation2d.fromDegrees(69), Rotation2d.fromDegrees(69))
//                    .splineTo(new Translation2d(Units.inches_to_meters(-17.8), Units.inches_to_meters(-128.9)));

            var path = builder.build();
            return generateTrajectory(path, config);
        }

        private Trajectory getThreeBallAutoPhase1Trajectory(TrajectoryConfig config) {
            var builder = new PathBuilder(
                    new edu.wpi.first.math.geometry.Translation2d(Units.inches_to_meters(-22.2), Units.inches_to_meters(-128.9)),
                    edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90))
                    .lineTo(new edu.wpi.first.math.geometry.Translation2d(Units.inches_to_meters(-100), Units.inches_to_meters(-78)),
                            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
            var path = builder.build();
            return generateTrajectory(path, config);
        }

        private Trajectory getThreeBallAutoPhase2Trajectory(TrajectoryConfig config) {
            var builder = new PathBuilder(
                    new edu.wpi.first.math.geometry.Translation2d(Units.inches_to_meters(-100), Units.inches_to_meters(-78)),
                    edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0))
                    .lineTo(new edu.wpi.first.math.geometry.Translation2d(Units.inches_to_meters(-18.5), Units.inches_to_meters(-48.2)),
                            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(69));
            var path = builder.build();
            return generateTrajectory(path, config);
        }

        private Trajectory getTestTrajectory(TrajectoryConfig config) {
            var builder = new SplinePathBuilder(
                    Translation2d.identity(), Rotation2d.identity(), Rotation2d.fromDegrees(180))
                    .splineTo(new Translation2d(Units.inches_to_meters(60), Units.inches_to_meters(-45)));

            var path = builder.build();
            return generateTrajectory(path, config);
        }

        private Trajectory getTestTrajectoryBack(TrajectoryConfig config) {
            var builder = new SplinePathBuilder(
                    Translation2d.identity(), Rotation2d.identity(), Rotation2d.fromDegrees(180))
                    .splineTo(new Translation2d(Units.inches_to_meters(-60), Units.inches_to_meters(45)));

            var path = builder.build();
            return generateTrajectory(path, config);
        }

        private Trajectory getTerminalTrajectory(TrajectoryConfig config) {
            var builder = new PathBuilder(
                    new edu.wpi.first.math.geometry.Translation2d(Units.inches_to_meters(-100), Units.inches_to_meters(-78)),
                    edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0))
                    .lineTo(new edu.wpi.first.math.geometry.Translation2d(Units.inches_to_meters(-260.3), Units.inches_to_meters(-107.0)),
                    edu.wpi.first.math.geometry.Rotation2d.fromDegrees(45));


            var path = builder.build();
            return generateTrajectory(path, config);
        }
    }
}
