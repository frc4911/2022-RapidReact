package frc.robot.paths;

import edu.wpi.first.wpilibj.Timer;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
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

import java.util.ArrayList;
import java.util.List;

import static libraries.cyberlib.trajectory.swerve.SwerveTrajectoryGenerator.create_trajectory;

public class SwerveDriveTrajectoryGenerator {
    private static SwerveDriveTrajectoryGenerator mInstance;
    private SwerveDriveTrajectoryGenerator.TrajectorySet mTrajectorySet = null;

    public static SwerveDriveTrajectoryGenerator getInstance() {
        if (mInstance == null) {
            mInstance = new SwerveDriveTrajectoryGenerator();
        }
        return mInstance;
    }

    private SwerveDriveTrajectoryGenerator() {
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

    public class TrajectorySet {

        public final Trajectory testTrajectory;
        public final Trajectory testTrajectoryBack;

        private TrajectorySet(TrajectoryConfig config) {
            // TODO: Implement deep clone so a trajectory generator function can freely modify the configuration.
            // NOTE: Constraints are not deep copied for now.
            testTrajectory = getTestTrajectory(config);
            testTrajectoryBack = getTestTrajectory(config);
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
    }
}
