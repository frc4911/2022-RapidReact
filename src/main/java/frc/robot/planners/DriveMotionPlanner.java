package frc.robot.planners;

import java.util.Optional;

import frc.robot.config.RobotConfiguration;
import frc.robot.config.SwerveConfiguration;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Pose2dWithCurvature;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.trajectory.Trajectory;
import libraries.cheesylib.trajectory.TrajectoryIterator;
import libraries.cheesylib.trajectory.timing.TimedState;
import libraries.cheesylib.util.CSVWritable;
import libraries.cheesylib.util.Util;
import libraries.cyberlib.control.HolonomicFeedforward;
import libraries.cyberlib.control.HolonomicTrajectoryFollower;
import libraries.cyberlib.control.PidGains;
import libraries.cyberlib.control.PurePursuitTrajectoryFollower;
import libraries.cyberlib.control.SwerveDriveFeedforwardGains;
import libraries.cyberlib.control.TrajectoryFollower;
import libraries.cyberlib.kinematics.ChassisSpeeds;
import libraries.cyberlib.utils.HolonomicDriveSignal;
import libraries.cyberlib.utils.RobotName;

public class DriveMotionPlanner implements CSVWritable {
    private HolonomicTrajectoryFollower follower;
    private HolonomicDriveSignal driveSignal = null;

    TrajectoryIterator<TimedState<Pose2dWithCurvature>> mCurrentTrajectory;

    public Trajectory<TimedState<Pose2dWithCurvature>> getTrajectory() {
        return mCurrentTrajectory.trajectory();
    }

    public double getRemainingProgress() {
        if (mCurrentTrajectory != null) {
            return mCurrentTrajectory.getRemainingProgress();
        }
        return 0.0;
    }

    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    public TimedState<Pose2dWithCurvature> mSetpoint = new TimedState<>(Pose2dWithCurvature.identity());
    Pose2d mError = Pose2d.identity();
    HolonomicDriveSignal mOutput = new HolonomicDriveSignal(Translation2d.identity(), 0.0, true);
    double currentTrajectoryLength = 0.0;

    double mDt = 0.0;
    double mSumDt;

    public SwerveConfiguration mSwerveConfiguration;

    public DriveMotionPlanner() {
        RobotConfiguration mRobotConfiguration = RobotConfiguration.getRobotConfiguration(RobotName.name);
        mSwerveConfiguration = mRobotConfiguration.getSwerveConfiguration();
        double transKP = 4.0;//1.4;
        double transKD = 0;//0.025;
        double rotKP = 0.0;
        double rotKD = 0;
        double ff0 = 0;//0.2;
        double ff1 = 0;//0.002;
        double ff2 = 0;//0.2;

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
        follower = new HolonomicTrajectoryFollower(
                new PidGains(transKP, 0.0, transKD),
                new PidGains(rotKP, 0.0, rotKD),
                new HolonomicFeedforward(new SwerveDriveFeedforwardGains(
                        ff0, // 0.042746,
                        ff1,
                        ff2)));
    }

    public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        mSumDt = 0;
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
        currentTrajectoryLength = trajectory.trajectory().getLastState().t();
        for (int i = 0; i < trajectory.trajectory().length(); ++i) {
            if (trajectory.trajectory().getState(i).velocity() > Util.kEpsilon) {
                mIsReversed = false;
                break;
            } else if (trajectory.trajectory().getState(i).velocity() < -Util.kEpsilon) {
                mIsReversed = true;
                break;
            }
        }
        follower.follow(mCurrentTrajectory);
    }

    public void reset() {
        mError = Pose2d.identity();
        mOutput = new HolonomicDriveSignal(Translation2d.identity(), 0.0, true);
        mLastTime = Double.POSITIVE_INFINITY;
        mSumDt = 0;
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

        if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;
        }

        mDt = timestamp - mLastTime;
        mSumDt += mDt;
        mLastTime = timestamp;

        if (!mCurrentTrajectory.isDone()) {
            var velocity = new Translation2d(chassisSpeeds.vxInMetersPerSecond, chassisSpeeds.vyInMetersPerSecond);

            Optional<HolonomicDriveSignal> trajectorySignal = follower.update(current_state,
                    velocity,
                    chassisSpeeds.omegaInRadiansPerSecond,
                    mLastTime,
                    mDt);

            mSetpoint = follower.getLastState();
            mError = current_state.inverse().transformBy(mSetpoint.state().getPose());

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
//mLastTime < mCurrentTrajectory.trajectory().getLastState().t()
    public boolean isDone() {
        return (mCurrentTrajectory != null && mCurrentTrajectory.isDone()) || (mSumDt>currentTrajectoryLength);
    }

    public Pose2d error() {
        return mError;
    }

    public TimedState<Pose2dWithCurvature> setpoint() {
        return mSetpoint;
    }
}
