package libraries.cheesylib.trajectory.timing;

import libraries.cheesylib.geometry.Pose2dWithCurvature;

public class CentripetalAccelerationConstraint implements TimingConstraint<Pose2dWithCurvature> {
    final double mMaxCentripetalAccel;

    public CentripetalAccelerationConstraint(final double max_centripetal_accel) {
        mMaxCentripetalAccel = max_centripetal_accel;
    }

    @Override
    public double getMaxVelocity(final Pose2dWithCurvature state) {
        /**
         * let A be the centripetal acceleration
         * let V be the max velocity
         * let C be the curvature of the path
         *
         * A = CV^2
         * A / C = V^2
         * sqrt(A / C) = V
         *
         * Curvature and max acceleration is always positive and we only expect a positive result
         * so plus-minus is not needed.
         *
         * Special case when following a line, centripetal acceleration is 0 so don't constrain velocity
         */
        if (state.getCurvature() == 0.0) {
            return MinMaxAcceleration.kNoLimits.max_acceleration();
        }

        return Math.sqrt(Math.abs(mMaxCentripetalAccel / state.getCurvature()));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(final Pose2dWithCurvature state, final double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }
}
