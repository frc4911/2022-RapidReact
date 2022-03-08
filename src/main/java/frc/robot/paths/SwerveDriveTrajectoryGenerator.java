package frc.robot.paths;

import edu.wpi.first.wpilibj.Timer;
import libraries.cheesylib.util.Units;
import libraries.cyberlib.paths.PathBuilder;
import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;
import libraries.cyberlib.trajectory.Trajectory;
import libraries.cyberlib.trajectory.TrajectoryConfig;
import libraries.cyberlib.trajectory.TrajectoryGenerator;

import java.util.List;

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

    public static Trajectory generateTrajectory(
            final List<PoseWithCurvatureAndOrientation> points,
            TrajectoryConfig config) {

        var trajectory = TrajectoryGenerator.generateTrajectoryFromPoints(points, config);
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
            var builder = new PathBuilder(
                    new edu.wpi.first.math.geometry.Translation2d(0.0, 0.0),
                    new edu.wpi.first.math.geometry.Rotation2d(),
                    new edu.wpi.first.math.geometry.Rotation2d())
                    .lineTo(new edu.wpi.first.math.geometry.Translation2d(
                            Units.inches_to_meters(60), Units.inches_to_meters(-45)))
                    .lineTo(new edu.wpi.first.math.geometry.Translation2d(
                            Units.inches_to_meters(100), Units.inches_to_meters(0)))
                    .lineTo(new edu.wpi.first.math.geometry.Translation2d(
                            Units.inches_to_meters(60), Units.inches_to_meters(0)))
                    .lineTo(new edu.wpi.first.math.geometry.Translation2d(
                            Units.inches_to_meters(0), Units.inches_to_meters(0)));

            var path = builder.build();
            return generateTrajectory(path.parameterize(), config);
        }

        private Trajectory getTestTrajectoryBack(TrajectoryConfig config) {
            var builder = new PathBuilder(
                    new edu.wpi.first.math.geometry.Translation2d(0.0, 0.0),
                    new edu.wpi.first.math.geometry.Rotation2d(),
                    new edu.wpi.first.math.geometry.Rotation2d())
                    .lineTo(new edu.wpi.first.math.geometry.Translation2d(
                            Units.inches_to_meters(60), Units.inches_to_meters(45)))
                    .lineTo(new edu.wpi.first.math.geometry.Translation2d(
                            Units.inches_to_meters(100), Units.inches_to_meters(0)))
                    .lineTo(new edu.wpi.first.math.geometry.Translation2d(
                            Units.inches_to_meters(60), Units.inches_to_meters(-45)))
                    .lineTo(new edu.wpi.first.math.geometry.Translation2d(
                            Units.inches_to_meters(0), Units.inches_to_meters(0)));

            var path = builder.build();
            return generateTrajectory(path.parameterize(), config);
       }
    }
}
