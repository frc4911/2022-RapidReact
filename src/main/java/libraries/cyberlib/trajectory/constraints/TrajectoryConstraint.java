package libraries.cyberlib.trajectory.constraints;


import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;

public interface TrajectoryConstraint {

    /**
     * Returns the max velocity given the current pose and curvature.
     *
     * @param state                                     The pose at the current point in the trajectory.
     * @param curvatureRadPerMeter                      The curvature at the current point in the trajectory.
     * @param translationalVelocityInMetersPerSecond    The translational velocity at the current point in the
     *                                                  trajectory before constraints are applied.
     * @param rotationalVelocityInRadiansPerSecond      The rotational velocity at the current point in the
     *                                                  trajectory before constraints are applied.
     * @return The absolute maximum velocity.
     */
    double getMaxVelocity(PoseWithCurvatureAndOrientation state,
                          double curvatureRadPerMeter,
                          double translationalVelocityInMetersPerSecond,
                          double rotationalVelocityInRadiansPerSecond);


    /**
     * Returns the minimum and maximum allowable acceleration for the trajectory
     * given pose, curvature, and speed.
     *
     * @param state                                     The pose at the current point in the trajectory.
     * @param curvatureRadPerMeter                      The curvature at the current point in the trajectory.
     * @param translationalVelocityInMetersPerSecond    The translational velocity at the current point in the
     *                                                  trajectory before constraints are applied.
     * @param rotationalVelocityInRadiansPerSecond      The rotational velocity at the current point in the
     *                                                  trajectory before constraints are applied.
     * @return The min and max acceleration bounds.
     */
    MinMaxAcceleration getMinMaxAcceleration(
            PoseWithCurvatureAndOrientation state,
            double curvatureRadPerMeter,
            double translationalVelocityInMetersPerSecond,
            double rotationalVelocityInRadiansPerSecond);

    /**
     * Represents a minimum and maximum acceleration.
     */
    public static class MinMaxAcceleration {
        protected final double mMinAcceleration;
        protected final double mMaxAcceleration;

        public static MinMaxAcceleration kNoLimits = new MinMaxAcceleration();

        public MinMaxAcceleration() {
            // No limits.
            mMinAcceleration = Double.NEGATIVE_INFINITY;
            mMaxAcceleration = Double.POSITIVE_INFINITY;
        }

        public MinMaxAcceleration(double minAcceleration, double maxAcceleration) {
            mMinAcceleration = minAcceleration;
            mMaxAcceleration = maxAcceleration;
        }

        public double getMinAcceleration() {
            return mMinAcceleration;
        }

        public double getMaxAcceleration() {
            return mMaxAcceleration;
        }

        public boolean isValid() {
            return getMinAcceleration() <= getMaxAcceleration();
        }
    }
}