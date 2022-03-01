package libraries.cyberlib.kinematics;

import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Twist2d;

/**
 * Class for swerve drive odometry. Odometry allows you to track the robot's
 * position on the field over a course of a match using readings from your
 * swerve drive encoders and swerve azimuth encoders.
 *
 * <p>
 * Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 */

public class SwerveDriveOdometry {
    private final SwerveDriveKinematics mKinematics;
    private Pose2d mPose;
    private double mPrevTimeSeconds = -1;

    private Rotation2d mGyroOffset;
    private Rotation2d mPreviousAngle;

    /**
     * Constructs a SwerveDriveOdometry object.
     *
     * @param kinematics  The swerve drive kinematics for your drivetrain.
     * @param gyroAngle   The angle reported by the gyroscope.
     * @param initialPose The starting position of the robot on the field.
     */
    public SwerveDriveOdometry(
            SwerveDriveKinematics kinematics, Rotation2d gyroAngle, Pose2d initialPose) {
        mKinematics = kinematics;
        mPose = initialPose;
        mGyroOffset = mPose.getRotation().minus(gyroAngle);
        mPreviousAngle = initialPose.getRotation();
    }

    /**
     * Constructs a SwerveDriveOdometry object with the default pose at the origin.
     *
     * @param kinematics The swerve drive kinematics for your drivetrain.
     * @param gyroAngle  The angle reported by the gyroscope.
     */
    public SwerveDriveOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle) {
        this(kinematics, gyroAngle, new Pose2d());
    }

    /**
     * Resets the robot's position on the field.
     *
     * <p>
     * The gyroscope angle does not need to be reset here on the user's robot code.
     * The library automatically takes care of offsetting the gyro angle.
     *
     * @param pose      The position on the field that your robot is at.
     * @param gyroAngle The angle reported by the gyroscope.
     */
    public void resetPosition(Pose2d pose, Rotation2d gyroAngle) {
        mPose = pose;
        mPreviousAngle = pose.getRotation();
        mGyroOffset = mPose.getRotation().minus(gyroAngle);
    }

    /**
     * Returns the position of the robot on the field.
     *
     * @return The pose of the robot (x and y are in unit of choice).
     */
    public Pose2d getPose() {
        return mPose;
    }

    /**
     * Updates the robot's position on the field using forward kinematics and
     * integration of the pose over time. This method takes in the current time as a
     * parameter to calculate period (difference between two timestamps). The period
     * is used to calculate the change in distance from a velocity. This also takes
     * in an angle parameter which is used instead of the angular rate that is
     * calculated from forward kinematics.
     *
     * @param currentTimeSeconds The current time in seconds.
     * @param gyroAngle          The angle reported by the gyroscope.
     * @param moduleStates       The current state of all swerve modules. Please
     *                           provide
     *                           the states in the same order in which you
     *                           instantiated your
     *                           SwerveDriveKinematics.
     * @return The new pose of the robot.
     */
    public Pose2d updateWithTime(
            double currentTimeSeconds, Rotation2d gyroAngle, SwerveModuleState... moduleStates) {
        double period = mPrevTimeSeconds >= 0 ? currentTimeSeconds - mPrevTimeSeconds : 0.0;
        mPrevTimeSeconds = currentTimeSeconds;

        var angle = gyroAngle.plus(mGyroOffset);

        var chassisState = mKinematics.toChassisSpeeds(moduleStates);
        var newPose = Pose2d.exp(
                new Twist2d(
                        chassisState.vxInMetersPerSecond * period,
                        chassisState.vyInMetersPerSecond * period,
                        angle.minus(mPreviousAngle).getRadians()));

        mPreviousAngle = angle;

        // The exp() implementation gives us the delta in poses. We need to add the
        // delta pose to the existing robot pose to accurately maintain the robot's pose
        // over time. Apply the transformation
        var pose = mPose.transformBy(newPose);
        mPose = new Pose2d(pose.getTranslation(), angle);
        return mPose;
    }
}
