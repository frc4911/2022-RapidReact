package libraries.cyberlib.trajectory.constraints;

import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;

import static libraries.cheesylib.util.Util.kEpsilon;


public class CentripetalAccelerationConstraint implements TrajectoryConstraint {
    private final double m_maxCentripetalAccelerationMetersPerSecondSq;

    /**
     * Constructs a centripetal acceleration constraint.
     *
     * @param maxCentripetalAccelerationMetersPerSecondSq The max centripetal acceleration.
     */
    public CentripetalAccelerationConstraint(double maxCentripetalAccelerationMetersPerSecondSq) {
        m_maxCentripetalAccelerationMetersPerSecondSq = maxCentripetalAccelerationMetersPerSecondSq;
    }

    /**
     * Returns the max velocity given the current pose and curvature.
     *
     * @param state                                     The pose at the current point in the trajectory.
     * @param curvatureInRadPerMeter                      The curvature at the current point in the trajectory.
     * @param translationalVelocityInMetersPerSecond    The translational velocity at the current point in the
     *                                                  trajectory before constraints are applied.
     * @param rotationalVelocityInRadiansPerSecond      The rotational velocity at the current point in the
     *                                                  trajectory before constraints are applied.
     * @return The absolute maximum velocity.
     */
    @Override
    public double getMaxVelocity(
            PoseWithCurvatureAndOrientation state,
            double curvatureInRadPerMeter,
            double translationalVelocityInMetersPerSecond,
            double rotationalVelocityInRadiansPerSecond)
    {
        // ac = v^2 / r
        // k (curvature) = 1 / r

        // therefore, ac = v^2 * k
        // ac / k = v^2
        // v = sqrt(ac / k)
        // let A be the centripetal acceleration
        // let V be the max velocity
        // let C be the curvature of the path
        // AC = V^2/R
        // C = 1 / R
        // A = CV^2
        // A / C = V^2
        // sqrt(A / C) = V
        //
        // Curvature and max acceleration is always positive, and we only expect a positive result
        // so plus-minus is not needed.
        //
        //  Special case when following a line, centripetal acceleration is 0 so don't constrain velocity

        double curvature = Math.abs(curvatureInRadPerMeter);
        if (curvature < kEpsilon) {
            return MinMaxAcceleration.kNoLimits.getMaxAcceleration();
        } else {
            return Math.sqrt(m_maxCentripetalAccelerationMetersPerSecondSq / curvature);
        }
    }

    /**
     * Returns the minimum and maximum allowable acceleration for the trajectory
     * given pose, curvature, and speed.
     *
     * @param state                                     The pose at the current point in the trajectory.
     * @param curvatureInRadPerMeter                      The curvature at the current point in the trajectory.
     * @param translationalVelocityInMetersPerSecond    The translational velocity at the current point in the
     *                                                  trajectory before constraints are applied.
     * @param rotationalVelocityInRadiansPerSecond      The rotational velocity at the current point in the
     *                                                  trajectory before constraints are applied.
     * @return The min and max acceleration bounds.
     */
    @Override
    public MinMaxAcceleration getMinMaxAcceleration(
            PoseWithCurvatureAndOrientation state,
            double curvatureInRadPerMeter,
            double translationalVelocityInMetersPerSecond,
            double rotationalVelocityInRadiansPerSecond)
    {
        // The acceleration of the robot has no impact on the centripetal acceleration
        // of the robot.
        return new MinMaxAcceleration();
    }
}
