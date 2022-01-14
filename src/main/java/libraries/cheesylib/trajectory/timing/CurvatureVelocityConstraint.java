package libraries.cheesylib.trajectory.timing;

import libraries.cheesylib.geometry.Pose2dWithCurvature;

public class CurvatureVelocityConstraint implements TimingConstraint<Pose2dWithCurvature>{
	final double mSwerveMaxSpeedInchesPerSecond;

	public CurvatureVelocityConstraint (final double swerveMaxSpeedInchesPerSecond) {
		mSwerveMaxSpeedInchesPerSecond = swerveMaxSpeedInchesPerSecond;
	}

	@Override
	public double getMaxVelocity(final Pose2dWithCurvature state){
		return mSwerveMaxSpeedInchesPerSecond / (1 + Math.abs(4.0*state.getCurvature()));//6.0
	}
	
	@Override
	public MinMaxAcceleration getMinMaxAcceleration(final Pose2dWithCurvature state, final double velocity){
		return MinMaxAcceleration.kNoLimits;
	}
	
}
