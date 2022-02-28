package libraries.cheesylib.trajectory;

import libraries.cheesylib.geometry.Pose2dWithCurvature;
import libraries.cheesylib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents the configuration for generating a trajectory. This class stores translational and rotational
 * start and end velocities, max velocities, max accelerations, custom constraints, and the reversed flag.
 *
 * <p>The class must be constructed with the max translational velocity and accelerations or
 * max translational and rotational velocity and accelerations. The other parameters
 * (start velocity, end velocity, constraints, reversed) have been defaulted to reasonable
 * values (0, 0, {}, false). These values can be changed via the setXXX methods.
// */
public class TrajectoryConfig {
    private final double mMaxVelocity;
    private final double mMaxAcceleration;
    private final double mMaxAngularVelocity;
    private final double mMaxAngularAcceleration;
    private final List<TimingConstraint<Pose2dWithCurvature>> mConstraints;
    private double mStartVelocity;
    private double mEndVelocity;
    private boolean mIsReversed;
    private double mStartAngularVelocity;
    private double mEndAngularVelocity;

    /**
     * Constructs the trajectory configuration class.
     *
     * @param maxVelocityMetersPerSecond       The max translational velocity for the trajectory.
     * @param maxAccelerationMetersPerSecondSq The max translational acceleration for the trajectory.
     */
    public TrajectoryConfig(
		final double maxVelocityMetersPerSecond, final double maxAccelerationMetersPerSecondSq) {
        this(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq, 0.0, 0.0);
    }

    /**
     * Constructs the trajectory configuration class.
     *
     * @param maxVelocityMetersPerSecond    The max translational velocity for the trajectory.
     * @param maxAccelerationMetersPerSecondSq The max translational acceleration for the trajectory.
     * @param maxAngularVelocityRadiansPerSecond       The max rotational velocity for the trajectory.
     * @param maxAngularAccelerationRadiansPerSecondSq The max rotational acceleration for the trajectory.
     */
    public TrajectoryConfig(final double maxVelocityMetersPerSecond,
                            final double maxAccelerationMetersPerSecondSq,
                            final double maxAngularVelocityRadiansPerSecond,
                            final double maxAngularAccelerationRadiansPerSecondSq) {
        mMaxVelocity = maxVelocityMetersPerSecond;
        mMaxAcceleration = maxAccelerationMetersPerSecondSq;
        mMaxAngularVelocity = maxAngularVelocityRadiansPerSecond;
        mMaxAngularAcceleration = maxAngularAccelerationRadiansPerSecondSq;
        mConstraints = new ArrayList<>();
    }

    /**
     * Create a shallow clone of a Trajectory config
     *
     * <p>NOTE: Constraints are not deep copied.
     * @param source the original TrajectoryConfig
     * @return a copy of the original
     */
    public static TrajectoryConfig fromTrajectoryConfig(TrajectoryConfig source
    ) {
        return new TrajectoryConfig(source.mMaxVelocity, source.mMaxAcceleration)
                .setReversed(source.isReversed())
                .setStartVelocity(source.getStartVelocity())
                .setEndVelocity(source.getEndVelocity())
                .setStartAngularVelocity(source.getStartAngularVelocity())
                .setEndAngularVelocity(source.getEndAngularVelocity())
                .addConstraints(source.getConstraints());
    }

    /**
     * Adds a user-defined constraint to the trajectory.
     *
     * @param constraint The user-defined constraint.
     * @return Instance of the current config object.
     */
    public TrajectoryConfig addConstraint(TimingConstraint<Pose2dWithCurvature> constraint) {
        mConstraints.add(constraint);
        return this;
    }

    /**
     * Adds all user-defined constraints from a list to the trajectory.
     * <p>
     * @param constraints List of user-defined constraints.
     * @return Instance of the current config object.
     */
    public TrajectoryConfig addConstraints(List<? extends TimingConstraint<Pose2dWithCurvature>> constraints) {
        mConstraints.addAll(constraints);
        return this;
    }

    /**
     * Returns the starting translational velocity of the trajectory.
     *
     * @return The starting translational velocity of the trajectory.
     */
    public double getStartVelocity() {
        return mStartVelocity;
    }

    /**
     * Sets the start translational velocity of the trajectory.
     *
     * @param startVelocityMetersPerSecond The start translational velocity of the trajectory.
     * @return Instance of the current config object.
     */
    public TrajectoryConfig setStartVelocity(final double startVelocityMetersPerSecond) {
        mStartVelocity = startVelocityMetersPerSecond;
    return this;
    }

    /**
     * Returns the end translational velocity of the trajectory.
     * <p>
     * @return The end translational velocity of the trajectory.
     */
    public double getEndVelocity() {
        return mEndVelocity;
    }

    /**
     * Sets the end translational velocity of the trajectory.
     *
     * @param endVelocityMetersPerSecond The end translational velocity of the trajectory.
     * @return Instance of the current config object.
     */
    public TrajectoryConfig setEndVelocity(final double endVelocityMetersPerSecond) {
        mEndVelocity = endVelocityMetersPerSecond;
    return this;
    }

    /**
     * Returns the starting rotational velocity of the trajectory.
     *
     * @return The starting rotational velocity of the trajectory.
     */
    public double getStartAngularVelocity() {
        return mStartAngularVelocity;
    }

    /**
     * Sets the starting rotational velocity of the trajectory.
     *
     * @param startAngularVelocityRadiansPerSecond The starting rotational velocity of the trajectory.
     * @return Instance of the current config object.
     */
    public TrajectoryConfig setStartAngularVelocity(final double startAngularVelocityRadiansPerSecond) {
        mStartAngularVelocity = startAngularVelocityRadiansPerSecond;
        return this;
    }

    /**
     * Returns the end rotational velocity of the trajectory.
     *
     * @return The end rotational velocity of the trajectory.
     */
    public double getEndAngularVelocity() {
        return mEndAngularVelocity;
    }

    /**
     * Sets the end rotational velocity of the trajectory.
     *
     * @param endAngularVelocityRadiansPerSecond The end rotational velocity of the trajectory.
     * @return Instance of the current config object.
     */
    public TrajectoryConfig setEndAngularVelocity(final double endAngularVelocityRadiansPerSecond) {
        mEndAngularVelocity = endAngularVelocityRadiansPerSecond;
        return this;
    }

    /**
    * Returns the maximum velocity of the trajectory.
    *
    * @return The maximum velocity of the trajectory.
    */
    public double getMaxVelocity() {
        return mMaxVelocity;
    }

    /**
     * Returns the maximum translational acceleration of the trajectory.
     *
     * @return The maximum translational acceleration of the trajectory.
     */
    public double getMaxAcceleration() {
        return mMaxAcceleration;
    }

    /**
     * Returns the maximum rotational velocity of the trajectory.
     *
     * @return The maximum rotational velocity of the trajectory.
     */
    public double getMaxAngularVelocity() {
        return mMaxAngularVelocity;
    }

    /**
     * Returns the maximum rotational acceleration of the trajectory.
     *
     * @return The maximum rotational acceleration of the trajectory.
     */
    public double getMaxAngularAcceleration() {
        return mMaxAngularAcceleration;
    }

    /**
     * Returns the user-defined constraints of the trajectory.
     *
     * @return The user-defined constraints of the trajectory.
     */
    public List<TimingConstraint<Pose2dWithCurvature>> getConstraints() {
        return mConstraints;
    }

    /**
     * Returns whether the trajectory is reversed or not.
     *
     * @return whether the trajectory is reversed or not.
     */
    public boolean isReversed() {
        return mIsReversed;
    }

    /**
     * Sets the reversed flag of the trajectory.
     *
     * @param reversed Whether the trajectory should be reversed or not.
     * @return Instance of the current config object.
     */
    public TrajectoryConfig setReversed(final boolean reversed) {
        mIsReversed = reversed;
        return this;
    }
}