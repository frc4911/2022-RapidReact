package libraries.cyberlib.trajectory.constraints;


import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;

public interface TrajectoryConstraint {

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
    double getMaxVelocity(PoseWithCurvatureAndOrientation state,
                          double curvatureInRadPerMeter,
                          double translationalVelocityInMetersPerSecond,
                          double rotationalVelocityInRadiansPerSecond);


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
    MinMaxAcceleration getMinMaxAcceleration(
            PoseWithCurvatureAndOrientation state,
            double curvatureInRadPerMeter,
            double translationalVelocityInMetersPerSecond,
            double rotationalVelocityInRadiansPerSecond);

    /**
     * Represents a minimum and maximum acceleration.
     */
    public static class MinMaxAcceleration {
        protected final double mMinAccelerationInMetersPerSecondSq;
        protected final double mMaxAccelerationInMetersPerSecondSq;

        public static MinMaxAcceleration kNoLimits = new MinMaxAcceleration();

        public MinMaxAcceleration() {
            // No limits.
            mMinAccelerationInMetersPerSecondSq = -Double.MAX_VALUE;
            mMaxAccelerationInMetersPerSecondSq = Double.MAX_VALUE;
        }

        public MinMaxAcceleration(double minAccelerationInMetersPerSecondSq, double maxAccelerationInMetersPerSecondSq) {
            mMinAccelerationInMetersPerSecondSq = minAccelerationInMetersPerSecondSq;
            mMaxAccelerationInMetersPerSecondSq = maxAccelerationInMetersPerSecondSq;
        }

        public double getMinAcceleration() {
            return mMinAccelerationInMetersPerSecondSq;
        }

        public double getMaxAcceleration() {
            return mMaxAccelerationInMetersPerSecondSq;
        }

        public boolean isValid() {
            return getMinAcceleration() <= getMaxAcceleration();
        }
    }
}