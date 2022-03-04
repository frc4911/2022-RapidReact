package frc.robot.paths;

import edu.wpi.first.wpilibj.Timer;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.util.Units;
import libraries.cyberlib.trajectory.Trajectory;
import libraries.cyberlib.trajectory.TrajectoryConfig;
import libraries.cyberlib.trajectory.swerve.OptimizationParameters;
import libraries.cyberlib.trajectory.swerve.SwerveTrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;

import static libraries.cyberlib.trajectory.TrajectoryGenerator.generateTrajectoryFromSplines;
import static libraries.cyberlib.trajectory.swerve.SwerveTrajectoryGenerator.createTrajectoryPoints;
import static libraries.cyberlib.trajectory.swerve.SwerveTrajectoryGenerator.generateSpline;

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
            final List<Translation2d> waypoints,
            TrajectoryConfig config,
            double startOrientation,
            double endOrientation) {
        List<SwerveTrajectoryGenerator.TrajectoryPoint> points = createTrajectoryPoints(waypoints);

        // Create an optimization parameter set per waypoint
        List<OptimizationParameters> optParams = new ArrayList<>();
        for (var i = 0; i < points.size(); i++) {
            optParams.add(new OptimizationParameters());
        }

        optParams.get(0).theta_lambda=1.0;
        optParams.get(1).theta_lambda=1.0;

        var splines = generateSpline(points, startOrientation, endOrientation, optParams);
//        var trajectory = SwerveTrajectoryGenerator.create_trajectory(
//                wayPoints, startOrientation, endOrientation, config, splines);
        var trajectory = generateTrajectoryFromSplines(splines, config);
        return trajectory;
    }

    public class TrajectorySet {

        public final Trajectory testTrajectory;
        public final Trajectory testTrajectoryBack;
        public final Trajectory twoBallAuto_toBallTrajectory;
        public final Trajectory twoBallAuto_toFenderTrajectory;


        private TrajectorySet(TrajectoryConfig config) {
            // TODO: Implement deep clone so a trajectory generator function can freely modify the configuration.
            // NOTE: Constraints are not deep copied for now.
            testTrajectory = getTestTrajectory(config);
            testTrajectoryBack = getTestTrajectory(config);
            twoBallAuto_toBallTrajectory = gettwoBallAuto_toBallTrajectory(config);
            twoBallAuto_toFenderTrajectory = gettwoBallAuto_toFenderTrajectory(config);
        }

        private Trajectory gettwoBallAuto_toBallTrajectory(TrajectoryConfig config) {
            List<Translation2d> waypoints = new ArrayList<>();
            waypoints.add(Translation2d.identity());
            waypoints.add(new Translation2d(Units.inches_to_meters(-90), Units.inches_to_meters(0)));
            var startOrientation = Math.toRadians(180);
            var endOrientation = Math.toRadians(180);
            return generateTrajectory(waypoints, config, startOrientation, endOrientation);
        }

        private Trajectory gettwoBallAuto_toFenderTrajectory(TrajectoryConfig config) {
            List<Translation2d> waypoints = new ArrayList<>();
            waypoints.add(new Translation2d(Units.inches_to_meters(-60), Units.inches_to_meters(0)));
            waypoints.add(new Translation2d(Units.inches_to_meters(20), Units.inches_to_meters(0)));
            var startOrientation = 0.0;
            var endOrientation = 0.0;
            return generateTrajectory(waypoints, config, startOrientation, endOrientation);
        }

        private Trajectory getTestTrajectory(TrajectoryConfig config) {
            List<Translation2d> waypoints = new ArrayList<>();
            waypoints.add(Translation2d.identity());
            waypoints.add(new Translation2d(Units.inches_to_meters(60), Units.inches_to_meters(-45)));
            waypoints.add(new Translation2d(Units.inches_to_meters(100), Units.inches_to_meters(0)));
            waypoints.add(new Translation2d(Units.inches_to_meters(60), Units.inches_to_meters(45)));
            waypoints.add(new Translation2d(Units.inches_to_meters(0), Units.inches_to_meters(0)));
            var startOrientation = 0.0;
            var endOrientation = Math.toRadians(180);
            return generateTrajectory(waypoints, config, startOrientation, endOrientation);
        }

        private Trajectory getTestTrajectoryBack(TrajectoryConfig config) {
            List<Translation2d> waypoints = new ArrayList<>();
            waypoints.add(Translation2d.identity());
            waypoints.add(new Translation2d(Units.inches_to_meters(60), Units.inches_to_meters(45)));
            waypoints.add(new Translation2d(Units.inches_to_meters(100), Units.inches_to_meters(0)));
            waypoints.add(new Translation2d(Units.inches_to_meters(60), Units.inches_to_meters(-45)));
            waypoints.add(new Translation2d(Units.inches_to_meters(0), Units.inches_to_meters(0)));
            var startOrientation = 0.0;
            var endOrientation = Math.toRadians(-180);
            return generateTrajectory(waypoints, config, startOrientation, endOrientation);
       }
    }
}
