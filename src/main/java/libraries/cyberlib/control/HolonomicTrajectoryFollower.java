package libraries.cyberlib.control;

import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Pose2dWithCurvature;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.trajectory.TrajectoryIterator;
import libraries.cheesylib.trajectory.timing.TimedState;
import libraries.cyberlib.utils.HolonomicDriveSignal;

public class HolonomicTrajectoryFollower extends TrajectoryFollower<HolonomicDriveSignal> {
    private final PidController mForwardController;
    private final PidController mStrafeController;
    private final PidController mRotationController;

    private HolonomicFeedforward feedforward;

    private TimedState<Pose2dWithCurvature> lastState = null;

    private boolean mFinished = false;

    public HolonomicTrajectoryFollower(PidGains translationGains, PidGains rotationGains,
            HolonomicFeedforward feedforward) {
        mForwardController = new PidController(translationGains);
        mStrafeController = new PidController(translationGains);
        mRotationController = new PidController(rotationGains);

        mRotationController.setContinuous(true);
        mRotationController.setInputRange(0.0, 2.0 * Math.PI);

        this.feedforward = feedforward;
    }

    @Override
    protected HolonomicDriveSignal calculateDriveSignal(Pose2d currentPose, Translation2d velocity,
            double rotationalVelocity,
            TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory, double time, double dt) {

        if (trajectory.isDone() || time > trajectory.trajectory().getLastState().t()) {
            mFinished = true;
            return new HolonomicDriveSignal(Translation2d.identity(), 0.0, false);
        }

        lastState = trajectory.advance(dt).state();

        Translation2d segmentVelocity = new Translation2d(
                lastState.state().getRotation().cos(),
                lastState.state().getRotation().sin()).scale(lastState.velocity());

        Translation2d segmentAcceleration = new Translation2d(
                lastState.state().getRotation().cos(),
                lastState.state().getRotation().sin()).scale(lastState.acceleration());

        Translation2d feedforwardVector = feedforward.calculateFeedforward(segmentVelocity, segmentAcceleration);

        mForwardController.setSetpoint(lastState.state().getPose().getTranslation().x());
        mStrafeController.setSetpoint(lastState.state().getPose().getTranslation().y());

        // TODO - make sure that pose rotation represents angular rotation in path
        // generation
        mRotationController.setSetpoint(lastState.state().getPose().getRotation().getRadians());

        return new HolonomicDriveSignal(
                new Translation2d(
                        mForwardController.calculate(currentPose.getTranslation().x(), dt) + feedforwardVector.x(),
                        mStrafeController.calculate(currentPose.getTranslation().y(), dt) + feedforwardVector.y()),
                mRotationController.calculate(currentPose.getRotation().getRadians(), dt), true);
    }

    public TimedState<Pose2dWithCurvature> getLastState() {
        return lastState;
    }

    @Override
    protected boolean isFinished() {
        return mFinished;
    }

    @Override
    protected void reset() {
        mForwardController.reset();
        mStrafeController.reset();
        mRotationController.reset();

        mFinished = false;
    }
}