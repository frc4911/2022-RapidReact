package libraries.cyberlib.trajectory.constraints;

import libraries.cyberlib.kinematics.ChassisSpeeds;
import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;
import libraries.cyberlib.kinematics.SwerveDriveKinematics;

/**
 * A class that enforces constraints on the swerve drive kinematics.
 * This can be used to ensure that the trajectory is constructed so that the
 * commanded velocities for all 4 wheels of the drivetrain stay below a certain
 * limit.
 */
public class SwerveDriveConstraint implements TrajectoryConstraint {
    private final double mMaxSpeedInMetersPerSecond;
    private final double mMaxAngularSpeedInRadiansPerSecond;
    private final SwerveDriveKinematics mKinematics;

    /**
     * Constructs a swerve drive dynamics constraint.
     *
     * @param maxSpeedInMetersPerSecond The max speed that a side of the robot can travel at.
     * @param maxAngularSpeedInRadiansPerSecond The max speed that a side of the robot can travel at.
     */
    public SwerveDriveConstraint(
			final SwerveDriveKinematics kinematics,
	        final double maxSpeedInMetersPerSecond,
	        final double maxAngularSpeedInRadiansPerSecond) {
        mMaxSpeedInMetersPerSecond = maxSpeedInMetersPerSecond;
        mMaxAngularSpeedInRadiansPerSecond = maxAngularSpeedInRadiansPerSecond;
        mKinematics = kinematics;
    }

    /**
     * Returns the max velocity given the current pose, trajectory curvature, the translational and
     * rotational velocities.
     *
     * @param state                                  The pose and curvature at the current point in the trajectory.
     * @param curvatureInRadPerMeter                   The curvature at the current point in the trajectory.
     * @param translationalVelocityInMetersPerSecond The translational velocity at the current point in the
     *                                               trajectory the before constraints are applied.
     * @param rotationalVelocityInRadiansPerSecond   The rotational velocity at the current point in the
     *                                               trajectory before constraints are applied.
     * @return The absolute maximum velocity.
     */
    @Override
    public double getMaxVelocity(
            PoseWithCurvatureAndOrientation state,
            double curvatureInRadPerMeter,
            double translationalVelocityInMetersPerSecond,
            double rotationalVelocityInRadiansPerSecond) {
        // Represents the velocity of the chassis in the x direction
        var xdVelocity = translationalVelocityInMetersPerSecond * state.poseMeters.getRotation().getCos();

        // Represents the velocity of the chassis in the y direction
        var ydVelocity = translationalVelocityInMetersPerSecond * state.poseMeters.getRotation().getSin();

        // Represents the angular velocity of the chassis
        var angularVelocity = Math.min(
                (translationalVelocityInMetersPerSecond * state.curvatureRadPerMeter) + state.angularVelocity,
                Math.min(rotationalVelocityInRadiansPerSecond, mMaxAngularSpeedInRadiansPerSecond));

        // Create an object to represent the current chassis speeds.
        var chassisSpeeds = new ChassisSpeeds(xdVelocity, ydVelocity, angularVelocity);

        // Get the wheel speeds and normalize them to within the max velocity.
        var wheelSpeeds = mKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(wheelSpeeds, mMaxSpeedInMetersPerSecond);

        // Convert normalized wheel speeds back to chassis speeds
        var normSpeeds = mKinematics.toChassisSpeeds(wheelSpeeds);

        // Return the new linear chassis speed.
        return Math.hypot(normSpeeds.vxInMetersPerSecond, normSpeeds.vyInMetersPerSecond);
    }

    /**
     * Returns the minimum and maximum allowable acceleration for the trajectory
     * given pose, curvature, and speed.
     *
     * @param state                                  The pose and curvature at the current point in the trajectory.
     * @param curvatureInRadPerMeter                   The curvature at the current point in the trajectory.
     * @param translationalVelocityInMetersPerSecond The translational velocity at the current point in the
     *                                               trajectory before constraints are applied.
     * @param rotationalVelocityInRadiansPerSecond   The rotational velocity at the current point in the
     *                                               trajectory before constraints are applied.
     * @return The min and max acceleration bounds.
     */
    @Override
    public MinMaxAcceleration getMinMaxAcceleration(
            PoseWithCurvatureAndOrientation state,
            double curvatureInRadPerMeter,
            double translationalVelocityInMetersPerSecond,
            double rotationalVelocityInRadiansPerSecond) {
        return MinMaxAcceleration.kNoLimits;
    }
}