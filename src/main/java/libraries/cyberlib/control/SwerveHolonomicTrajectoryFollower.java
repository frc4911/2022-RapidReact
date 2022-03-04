package libraries.cyberlib.control;

import libraries.cheesylib.geometry.Translation2d;
import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;
import libraries.cyberlib.trajectory.Trajectory;
import libraries.cyberlib.utils.HolonomicDriveSignal;

public class SwerveHolonomicTrajectoryFollower extends SwerveTrajectoryFollower<HolonomicDriveSignal> {
    private final PidController mForwardController;
    private final PidController mStrafeController;
    private final PidController mRotationController;

    private HolonomicFeedforward feedforward;

    private Trajectory.State lastState = null;

    private boolean mFinished = false;

    public SwerveHolonomicTrajectoryFollower(PidGains translationGains, PidGains rotationGains,
                                       HolonomicFeedforward feedforward) {
        mForwardController = new PidController(translationGains);
        mStrafeController = new PidController(translationGains);
        mRotationController = new PidController(rotationGains);

        mRotationController.setContinuous(true);
        mRotationController.setInputRange(0.0, 2.0 * Math.PI);

        this.feedforward = feedforward;
    }

    @Override
    protected HolonomicDriveSignal calculateDriveSignal(PoseWithCurvatureAndOrientation currentPose, Translation2d velocity,
                                                        double rotationalVelocity,
                                                        Trajectory trajectory, double time, double dt) {

        if (time > trajectory.getTotalTimeSeconds()) {
            mFinished = true;
            return new HolonomicDriveSignal(Translation2d.identity(), 0.0, false);
        }

        lastState = trajectory.sample(dt);

        Translation2d segmentVelocity = new Translation2d(
                lastState.poseMeters.getRotation().getCos(),
                lastState.poseMeters.getRotation().getSin()).scale(lastState.velocityMetersPerSecond);

        Translation2d segmentAcceleration = new Translation2d(
                lastState.poseMeters.getRotation().getCos(),
                lastState.poseMeters.getRotation().getSin()).scale(lastState.accelerationMetersPerSecondSq);

        Translation2d feedforwardVector = feedforward.calculateFeedforward(segmentVelocity, segmentAcceleration);

        mForwardController.setSetpoint(lastState.poseMeters.getX());
        mStrafeController.setSetpoint(lastState.poseMeters.getY());

        // TODO - make sure that pose rotation represents angular rotation in path
        // generation
        mRotationController.setSetpoint(lastState.orientationRadians);

        return new HolonomicDriveSignal(
                new Translation2d(
                        mForwardController.calculate(currentPose.poseMeters.getX(), dt) + feedforwardVector.x(),
                        mStrafeController.calculate(currentPose.poseMeters.getY(), dt) + feedforwardVector.y()),
                mRotationController.calculate(currentPose.orientation, dt), true);
    }

    @Override
    public Trajectory.State getLastState() {
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