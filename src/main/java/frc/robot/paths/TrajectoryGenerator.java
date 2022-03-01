package frc.robot.paths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.planners.DriveMotionPlanner;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Pose2dWithCurvature;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.trajectory.Trajectory;
import libraries.cheesylib.trajectory.TrajectoryUtil;
import libraries.cheesylib.trajectory.timing.CentripetalAccelerationConstraint;
import libraries.cheesylib.trajectory.timing.TimedState;
import libraries.cheesylib.trajectory.timing.TimingConstraint;
import libraries.cheesylib.util.Units;

public class TrajectoryGenerator {
    private static double kMaxVelocity = Units.inches_to_meters(120.0);
    private static double kMaxAccel = Units.inches_to_meters(60.0); // 120.0;
    private static double kMaxDecel = Units.inches_to_meters(72.0); // 72.0;
    private static double kMaxCentriptalAccel = kMaxVelocity * kMaxVelocity; // assume unit radius of 1
    private static final double kMaxVoltage = 9.0;

    private final static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
        kMaxVelocity = mMotionPlanner.mSwerveConfiguration.maxSpeedInMetersPerSecond;
        kMaxAccel = mMotionPlanner.mSwerveConfiguration.maxAccellerationInMetersPerSecondSq;
        kMaxDecel = mMotionPlanner.mSwerveConfiguration.maxAccellerationInMetersPerSecondSq;
        kMaxCentriptalAccel = mMotionPlanner.mSwerveConfiguration.kMaxCentriptalAccelerationInMetersPerSecondSq;
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println(
                    "Finished trajectory generation in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel, // meters/s
            double max_accel, // meters/s^2
            double max_decel,
            double max_voltage,
            double default_vel,
            int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_decel,
                max_voltage,
                default_vel, slowdown_chunks);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel, // meters/s
            double end_vel, // meters/s
            double max_vel, // meters/s
            double max_accel, // meters/s^2
            double max_decel,
            double max_voltage,
            double default_vel,
            int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel,
                max_accel, max_decel, max_voltage,
                default_vel, slowdown_chunks);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle
    // of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the right.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON LEFT! (mirrored about +x
    // axis for RIGHT)
    static final Pose2d autoStartingPose = new Pose2d(
            Constants.kRobotLeftStartingPose.getTranslation().translateBy(new Translation2d(/*-0.5*/0.0, 0.0)),
            Rotation2d.fromDegrees(-90.0));

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> left) {
                this.left = left;
                this.right = TrajectoryUtil.mirrorTimed(left, left.defaultVelocity());
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        public final MirroredTrajectory testTrajectory;
        public final MirroredTrajectory testTrajectoryBack;
        public final MirroredTrajectory twoBallAuto_toBallTrajectory;
        public final MirroredTrajectory twoBallAuto_toFenderTrajectory;


        private TrajectorySet() {
            testTrajectory = new MirroredTrajectory(getTestTrajectory());
            testTrajectoryBack = new MirroredTrajectory(getTestTrajectoryBack());
            twoBallAuto_toBallTrajectory = new MirroredTrajectory(gettwoBallAuto_toBallTrajectory());
            twoBallAuto_toFenderTrajectory = new MirroredTrajectory(gettwoBallAuto_toFenderTrajectory());
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> gettwoBallAuto_toBallTrajectory() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(Units.inches_to_meters(-90), Units.inches_to_meters(0), Rotation2d.fromDegrees(180)));
            return generateTrajectory(false, waypoints,
                    Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentriptalAccel)),
                    Units.inches_to_meters(80.0), kMaxAccel, kMaxDecel, kMaxVoltage, Units.inches_to_meters(80.0), 1);
                    //  kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, kMaxVelocity, 1);
                //    Units.inches_to_meters(3.0), Units.inches_to_meters(30.0), Units.inches_to_meters(30.0), kMaxVoltage, Units.inches_to_meters(3.0), 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> gettwoBallAuto_toFenderTrajectory() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Units.inches_to_meters(-60), Units.inches_to_meters(0), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(Units.inches_to_meters(20), Units.inches_to_meters(0), Rotation2d.fromDegrees(0)));
            return generateTrajectory(false, waypoints,
                    Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentriptalAccel)),
                    Units.inches_to_meters(80.0), kMaxAccel, kMaxDecel, kMaxVoltage, Units.inches_to_meters(80.0), 1);
                    //  kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, kMaxVelocity, 1);
                //    Units.inches_to_meters(3.0), Units.inches_to_meters(30.0), Units.inches_to_meters(30.0), kMaxVoltage, Units.inches_to_meters(3.0), 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestTrajectory() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0)));
            waypoints.add(
                    new Pose2d(Units.inches_to_meters(60), Units.inches_to_meters(-45), Rotation2d.fromDegrees(0)));
            waypoints.add(
                    new Pose2d(Units.inches_to_meters(100), Units.inches_to_meters(0), Rotation2d.fromDegrees(90)));
            waypoints.add(
                    new Pose2d(Units.inches_to_meters(60), Units.inches_to_meters(45), Rotation2d.fromDegrees(180)));
            waypoints
                    .add(new Pose2d(Units.inches_to_meters(0), Units.inches_to_meters(0), Rotation2d.fromDegrees(180)));
            return generateTrajectory(false, waypoints,
                    Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentriptalAccel)),
                    Units.inches_to_meters(80.0), kMaxAccel, kMaxDecel, kMaxVoltage, Units.inches_to_meters(80.0), 1);
            // kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, kMaxVelocity, 1);
            // Units.inches_to_meters(3.0), Units.inches_to_meters(30.0),
            // Units.inches_to_meters(30.0), kMaxVoltage, Units.inches_to_meters(3.0), 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestTrajectoryBack() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0)));
            waypoints
                    .add(new Pose2d(Units.inches_to_meters(60), Units.inches_to_meters(45), Rotation2d.fromDegrees(0)));
            waypoints.add(
                    new Pose2d(Units.inches_to_meters(100), Units.inches_to_meters(0), Rotation2d.fromDegrees(-90)));
            waypoints.add(
                    new Pose2d(Units.inches_to_meters(60), Units.inches_to_meters(-45), Rotation2d.fromDegrees(-180)));
            waypoints.add(
                    new Pose2d(Units.inches_to_meters(0), Units.inches_to_meters(0), Rotation2d.fromDegrees(-180)));
            return generateTrajectory(false, waypoints,
                    Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentriptalAccel)),
                    Units.inches_to_meters(80.0), kMaxAccel, kMaxDecel, kMaxVoltage, Units.inches_to_meters(80.0), 1);
            // kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, kMaxVelocity, 1);
            // Units.inches_to_meters(3.0), Units.inches_to_meters(30.0),
            // Units.inches_to_meters(30.0), kMaxVoltage, Units.inches_to_meters(3.0), 1);
        }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getTestTrajectoryBack() {
        // List<Pose2d> waypoints = new ArrayList<>();
        // waypoints.add(new Pose2d(Units.inches_to_meters(-120),
        // Units.inches_to_meters(120), Rotation2d.fromDegrees(90)));
        // waypoints.add(new Pose2d(Translation2d.identity(),
        // Rotation2d.fromDegrees(180)));
        // return generateTrajectory(true, waypoints, Arrays.asList(new
        // CentripetalAccelerationConstraint(60)),
        // kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, kMaxVelocity, 1);
        // }
    }
}
