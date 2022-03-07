package frc.robot.planners;

import frc.robot.config.RobotConfiguration;
import frc.robot.config.SwerveConfiguration;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.util.CSVWritable;
import libraries.cheesylib.util.Util;
import libraries.cyberlib.control.*;
import libraries.cyberlib.kinematics.ChassisSpeeds;
import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;
import libraries.cyberlib.trajectory.Trajectory;
import libraries.cyberlib.utils.HolonomicDriveSignal;
import libraries.cyberlib.utils.RobotName;

import java.util.Optional;

public class SwerveDriveMotionPlanner implements CSVWritable {
    private SwerveHolonomicTrajectoryFollower follower;
    private HolonomicDriveSignal driveSignal = null;

    Trajectory mCurrentTrajectory;

    public Trajectory getTrajectory() {
        return mCurrentTrajectory;
    }

//    public double getRemainingProgress() {
//        if (mCurrentTrajectory != null) {
//            return mCurrentTrajectory.getRemainingProgress();
//        }
//        return 0.0;
//    }

    private boolean mIsReversed = false;
    private double mLastTime = Double.POSITIVE_INFINITY;
    public Trajectory.State mSetpoint = new Trajectory.State(
            0.0,
            0.0,
            0.0,
            new edu.wpi.first.math.geometry.Pose2d(),
            0.0,
            0.0,
            0.0,
            0.0);
    private Pose2d mError = Pose2d.identity();
//    private HolonomicDriveSignal mOutput = new HolonomicDriveSignal(Translation2d.identity(), 0.0, true);
    private double mCurrentTrajectoryTotalTime = 0.0;

    private double mDt = 0.0;
    private double mSumDt = 0.0;

    public SwerveConfiguration mSwerveConfiguration;

    public SwerveDriveMotionPlanner() {
        RobotConfiguration mRobotConfiguration = RobotConfiguration.getRobotConfiguration(RobotName.name);
        mSwerveConfiguration = mRobotConfiguration.getSwerveConfiguration();
        double transKP = .6;
        double transKD = 0.025;
        double rotKP = 0;
        double rotKD = 0;
        double ff0 = 0.1;
        double ff1 = 0.1;
        double ff2 = 0;

        // transKP = SmartDashboard.getNumber("transKP", -1);
        // if (transKP == -1){
        // SmartDashboard.putNumber("transKP", 0);
        // transKP = 0;
        // }

        // transKD = SmartDashboard.getNumber("transKD", -1);
        // if (transKD == -1){
        // SmartDashboard.putNumber("transKD", 0);
        // transKD = 0;
        // }

        // rotKP = SmartDashboard.getNumber("rotKP", -1);
        // if (rotKP == -1){
        // SmartDashboard.putNumber("rotKP", 0);
        // rotKP = 0;
        // }

        // rotKD = SmartDashboard.getNumber("rotKD", -1);
        // if (rotKD == -1){
        // SmartDashboard.putNumber("rotKD", 0);
        // rotKD = 0;
        // }

        // ff0 = SmartDashboard.getNumber("ff0", -1);
        // if (ff0 == -1){
        // SmartDashboard.putNumber("ff0", 0);
        // ff0 = 0;
        // }

        // ff1 = SmartDashboard.getNumber("ff1", -1);
        // if (ff1 == -1){
        // SmartDashboard.putNumber("ff1", 0);
        // ff1 = 0;
        // }

        // ff2 = SmartDashboard.getNumber("ff2", -1);
        // if (ff2 == -1){
        // SmartDashboard.putNumber("ff2", 0);
        // ff2 = 0;
        // }

        System.out.println("transKP = " + transKP);
        System.out.println("transKD = " + transKD);
        System.out.println("rotKP = " + rotKP);
        System.out.println("rotKD = " + rotKD);
        System.out.println("ff0 = " + ff0);
        System.out.println("ff1 = " + ff1);
        System.out.println("ff2 = " + ff2);

        // TODO: Make these constants
        // follower = new HolonomicTrajectoryFollower(
        // new PidGains(0.4, 0.0, 0.025),
        // new PidGains(5.0, 0.0, 0.0),
        // new HolonomicFeedforward(new SwerveDriveFeedforwardGains(
        // 0.289, //0.042746,
        // 0.0032181,
        // 0.30764
        // )));
        System.out.println("applied----------------------------------------");
        follower = new SwerveHolonomicTrajectoryFollower(
                new PidGains(transKP, 0.0, transKD),
                new PidGains(rotKP, 0.0, rotKD),
                new HolonomicFeedforward(new SwerveDriveFeedforwardGains(
                        ff0, // 0.042746,
                        ff1,
                        ff2)));
    }

    public void setTrajectory(final Trajectory trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getStates().get(0);
        mCurrentTrajectoryTotalTime = trajectory.getTotalTimeSeconds();
        mSumDt = 0.0;
        for (int i = 0; i < trajectory.getStates().size(); ++i) {
            if (trajectory.getStates().get(i).velocityMetersPerSecond > Util.kEpsilon) {
                mIsReversed = false;
                break;
            } else if (trajectory.getStates().get(i).velocityMetersPerSecond < -Util.kEpsilon) {
                mIsReversed = true;
                break;
            }
        }
        follower.follow(mCurrentTrajectory);
    }

    public void reset() {
        mError = Pose2d.identity();
//        mOutput = new HolonomicDriveSignal(Translation2d.identity(), 0.0, true);
        mLastTime = Double.POSITIVE_INFINITY;
    }


    @Override
    public String toCSV() {
        // return mOutput.toCSV();
        return "";
    }

    public HolonomicDriveSignal update(double timestamp, Pose2d current_state, ChassisSpeeds chassisSpeeds) {
        HolonomicDriveSignal driveSignal;

        if (mCurrentTrajectory == null) {
            return null;
        }

        if (mCurrentTrajectory.getTotalTimeSeconds() == 0.0 && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;
        }

        mDt = timestamp - mLastTime;
        mSumDt += mDt;
        mLastTime = timestamp;

        if (mCurrentTrajectory.getTotalTimeSeconds() <= timestamp) {
            var velocity = new Translation2d(chassisSpeeds.vxInMetersPerSecond, chassisSpeeds.vyInMetersPerSecond);

            // NOTE - Conversions between geometry systems is required until we switch completely to wpilib.
            var current = new PoseWithCurvatureAndOrientation(
                    new edu.wpi.first.math.geometry.Pose2d(
                            new edu.wpi.first.math.geometry.Translation2d(
                                    current_state.getTranslation().x(),
                                    current_state.getTranslation().y()),
                            new edu.wpi.first.math.geometry.Rotation2d(
                                    current_state.getRotation().getRadians())),
                    0.0,
                    chassisSpeeds.omegaInRadiansPerSecond,
                    0.0,
                    0.0);
            Optional<HolonomicDriveSignal> trajectorySignal = follower.update(current,
                    velocity,
                    chassisSpeeds.omegaInRadiansPerSecond,
                    mLastTime,
                    mDt);

            mSetpoint = follower.getLastState();

            // NOTE - Conversions between geometry systems is required until we switch completely to wpilib.
            var setPointPose = new Pose2d(
                    new Translation2d(mSetpoint.poseMeters.getX(), mSetpoint.poseMeters.getX()),
                    Rotation2d.fromRadians(mSetpoint.poseMeters.getRotation().getRadians()));

            mError = current_state.inverse().transformBy(setPointPose);

            if (trajectorySignal.isPresent()) {
                driveSignal = trajectorySignal.get();
                // Scale inputs as Swerve scales them.
                driveSignal = new HolonomicDriveSignal(
                        driveSignal.getTranslation().scale(1 / mSwerveConfiguration.maxSpeedInMetersPerSecond),
                        driveSignal.getRotation() / mSwerveConfiguration.maxSpeedInRadiansPerSecond,
                        driveSignal.isFieldOriented());
            } else {
                driveSignal = this.driveSignal;
            }
            return driveSignal;
        }

        return null;
    }

    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.getTotalTimeSeconds() < mSumDt;
    }

    public Pose2d error() {
        return mError;
    }

    public Trajectory.State setpoint() {
        return mSetpoint;
    }
}
