package frc.robot.paths;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Pose2dWithCurvature;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.spline.SplineGenerator;
import libraries.cheesylib.trajectory.Trajectory;
import libraries.cheesylib.trajectory.DistanceView;
import libraries.cheesylib.trajectory.TrajectoryConfig;
import libraries.cheesylib.trajectory.TrajectoryUtil;
import libraries.cheesylib.trajectory.timing.TimedState;
import libraries.cheesylib.trajectory.timing.TimingUtil;
import libraries.cheesylib.util.Units;

public class TrajectoryGenerator {
    private static TrajectoryGenerator mInstance;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        if (mInstance == null) {
            mInstance = new TrajectoryGenerator();
        }
        return mInstance;
    }

    private TrajectoryGenerator() {
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON LEFT! (mirrored about +x
    // axis for RIGHT)

    static final Pose2d autoStartingPose = new Pose2d(
            Constants.kRobotLeftStartingPose.getTranslation().translateBy(new Translation2d(/*-0.5*/0.0, 0.0)),
            Rotation2d.fromDegrees(-90.0));
    public void generateTrajectories(TrajectoryConfig config) {
        if (mTrajectorySet == null) {
            double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet(config);
            System.out.println(
                    "Finished trajectory generation in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public static Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            final List<Pose2d> waypoints,
            TrajectoryConfig config) {
        List<Pose2d> waypoints_maybe_flipped = waypoints;
        final Pose2d flip = Pose2d.fromRotation(new Rotation2d(-1, 0, false));

        // TODO re-architect the spline generator to support reverse.
        if (config.isReversed()) {
            waypoints_maybe_flipped = new ArrayList<>(waypoints.size());
            for (int i = 0; i < waypoints.size(); ++i) {
                waypoints_maybe_flipped.add(waypoints.get(i).transformBy(flip));
            }
        }

        // Create a trajectory from splines.
        Trajectory<Pose2dWithCurvature> trajectory =
                TrajectoryUtil.trajectoryFromSplineWaypoints(waypoints_maybe_flipped);

        if (config.isReversed()) {
            List<Pose2dWithCurvature> flipped = new ArrayList<>(trajectory.length());
            for (int i = 0; i < trajectory.length(); ++i) {
                flipped.add(new Pose2dWithCurvature(
                        trajectory.getState(i).getPose().transformBy(flip),
                        -trajectory.getState(i).getCurvature(),
                        trajectory.getState(i).getDCurvatureDs()));
            }
            trajectory = new Trajectory<>(flipped);
        }

        // NOTE: While trajectory configurations can be overridden here, remember to push back to SwerveConfiguration.

        // Generate the timed trajectory.
        Trajectory<TimedState<Pose2dWithCurvature>> timed_trajectory = TimingUtil.timeParameterizeTrajectory(
                config.isReversed(),
                new DistanceView<>(trajectory),
                SplineGenerator.kMaxDX,
                config.getConstraints(),
                config.getStartVelocity(),
                config.getEndVelocity(),
                config.getMaxVelocity(),
                config.getMaxAcceleration());

        return timed_trajectory;
    }

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> left) {
                this.left = left;
                this.right = TrajectoryUtil.mirrorTimed(left);
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        public final MirroredTrajectory testTrajectory;
        public final MirroredTrajectory testTrajectoryBack;
        public final MirroredTrajectory forwardTrajectory;
        public final MirroredTrajectory backwardTrajectory;
        public final MirroredTrajectory twoBallAuto_toBallTrajectory;
        public final MirroredTrajectory twoBallAuto_toFenderTrajectory;
        public final MirroredTrajectory threeBallAuto0Trajectory;
        public final MirroredTrajectory threeBallAuto1Trajectory;
        public final MirroredTrajectory threeBallAuto2Trajectory;


        private TrajectorySet(TrajectoryConfig config) {
            // TODO: Implement deep clone so a trajectory generator function can freely modify the configuration.
            // NOTE: Constraints are not deep copied for now.
            testTrajectory = new MirroredTrajectory(
                    getTestTrajectory(TrajectoryConfig.fromTrajectoryConfig(config)));
            testTrajectoryBack = new MirroredTrajectory(
                    getTestTrajectoryBack(TrajectoryConfig.fromTrajectoryConfig(config)));
            forwardTrajectory = new MirroredTrajectory(
                    getForwardTrajectory(TrajectoryConfig.fromTrajectoryConfig(config)));
            backwardTrajectory = new MirroredTrajectory(
                    getBackwardTrajectory(TrajectoryConfig.fromTrajectoryConfig(config)));
            twoBallAuto_toBallTrajectory = new MirroredTrajectory(
                    gettwoBallAuto_toBallTrajectory(TrajectoryConfig.fromTrajectoryConfig(config)));
            twoBallAuto_toFenderTrajectory = new MirroredTrajectory(
                    gettwoBallAuto_toFenderTrajectory(TrajectoryConfig.fromTrajectoryConfig(config)));
            threeBallAuto0Trajectory = new MirroredTrajectory(
                    getThreeBallAutoPhase0Trajectory(TrajectoryConfig.fromTrajectoryConfig(config)));
            threeBallAuto1Trajectory = new MirroredTrajectory(
                    getThreeBallAutoPhase1Trajectory(TrajectoryConfig.fromTrajectoryConfig(config)));
            threeBallAuto2Trajectory = new MirroredTrajectory(
                    getThreeBallAutoPhase2Trajectory(TrajectoryConfig.fromTrajectoryConfig(config)));
            }

        private Trajectory<TimedState<Pose2dWithCurvature>> getForwardTrajectory(TrajectoryConfig config) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(Units.inches_to_meters(110), Units.inches_to_meters(0), Rotation2d.fromDegrees(0)));
            return generateTrajectory(waypoints, config);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBackwardTrajectory(TrajectoryConfig config) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(Units.inches_to_meters(-120), Units.inches_to_meters(0), Rotation2d.fromDegrees(180)));
            return generateTrajectory(waypoints, config);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> gettwoBallAuto_toBallTrajectory(TrajectoryConfig config) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(Units.inches_to_meters(-80), Units.inches_to_meters(36), Rotation2d.fromDegrees(180)));
            return generateTrajectory(waypoints, config);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> gettwoBallAuto_toFenderTrajectory(TrajectoryConfig config) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Units.inches_to_meters(-80), Units.inches_to_meters(36), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(Units.inches_to_meters(-5), Units.inches_to_meters(-5), Rotation2d.fromDegrees(0)));
            return generateTrajectory(waypoints, config);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThreeBallAutoPhase0Trajectory(TrajectoryConfig config) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Units.inches_to_meters(0), Units.inches_to_meters(0), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(Units.inches_to_meters(-96), Units.inches_to_meters(-36), Rotation2d.fromDegrees(180)));
            return generateTrajectory(waypoints, config);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThreeBallAutoPhase1Trajectory(TrajectoryConfig config) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Units.inches_to_meters(-96), Units.inches_to_meters(-36), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(Units.inches_to_meters(-70), Units.inches_to_meters(82), Rotation2d.fromDegrees(180)));
            return generateTrajectory(waypoints, config);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThreeBallAutoPhase2Trajectory(TrajectoryConfig config) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Units.inches_to_meters(-70), Units.inches_to_meters(82), Rotation2d.fromDegrees(-40)));
            waypoints.add(new Pose2d(Units.inches_to_meters(-10), Units.inches_to_meters(0), Rotation2d.fromDegrees(0)));
            return generateTrajectory(waypoints, config);
        }


        // TEST
        private Trajectory<TimedState<Pose2dWithCurvature>> getTestTrajectory(TrajectoryConfig config) {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(Units.inches_to_meters(60), Units.inches_to_meters(0), Rotation2d.fromDegrees(0)));
            // waypoints.add(new Pose2d(Units.inches_to_meters(60), Units.inches_to_meters(-45), Rotation2d.fromDegrees(0)));
            // waypoints.add(new Pose2d(Units.inches_to_meters(100), Units.inches_to_meters(0), Rotation2d.fromDegrees(90)));
            // waypoints.add(new Pose2d(Units.inches_to_meters(60), Units.inches_to_meters(45), Rotation2d.fromDegrees(180)));
            // waypoints.add(new Pose2d(Units.inches_to_meters(0), Units.inches_to_meters(0), Rotation2d.fromDegrees(180)));
            return generateTrajectory(waypoints, config);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestTrajectoryBack(TrajectoryConfig config) {
            List<Pose2d> waypoints = new ArrayList<>();
            // waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0)));
            // waypoints.add(new Pose2d(Units.inches_to_meters(120), Units.inches_to_meters(0), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(Units.inches_to_meters(60), Units.inches_to_meters(0), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            // waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0)));
            // waypoints.add(new Pose2d(Units.inches_to_meters(60), Units.inches_to_meters(45), Rotation2d.fromDegrees(0)));
            // waypoints.add(new Pose2d(Units.inches_to_meters(100), Units.inches_to_meters(0), Rotation2d.fromDegrees(-90)));
            // waypoints.add(new Pose2d(Units.inches_to_meters(60), Units.inches_to_meters(-45), Rotation2d.fromDegrees(-180)));
            // waypoints.add(new Pose2d(Units.inches_to_meters(0), Units.inches_to_meters(0), Rotation2d.fromDegrees(-180)));
            return generateTrajectory(waypoints, config);
        }
    }
}
