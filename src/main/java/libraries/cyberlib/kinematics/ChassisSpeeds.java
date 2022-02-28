package libraries.cyberlib.kinematics;

import libraries.cheesylib.geometry.Rotation2d;

/**
 * Represents the speed of a robot chassis. Although this struct contains
 * similar members compared to a Twist2d, they do NOT represent the same thing.
 * Whereas a Twist2d represents a change in pose w.r.t to the robot frame of
 * reference, this ChassisSpeeds struct represents a velocity w.r.t to the robot
 * frame of reference.
 *
 * <p>
 * A strictly non-holonomic drivetrain, such as a differential drive, should
 * never have a dy component because it can never move sideways. Holonomic
 * drivetrains such as swerve and mecanum will often have all three components.
 */
public class ChassisSpeeds {
    /**
     * Represents forward velocity w.r.t the robot frame of reference. (Fwd is +)
     */
    public double vxInMetersPerSecond;

    /**
     * Represents sideways velocity w.r.t the robot frame of reference. (Left is +)
     */
    public double vyInMetersPerSecond;

    /**
     * Represents the angular velocity of the robot frame. (CCW is +)
     */
    public double omegaInRadiansPerSecond;

    /**
     * Constructs a ChassisSpeeds with zeros for dx, dy, and theta.
     */
    public ChassisSpeeds() {
    }

    /**
     * Constructs a ChassisSpeeds object.
     * x
     * 
     * @param vxInMetersPerSecond     Forward velocity in meters per second.
     * @param vyInMetersPerSecond     Sideways velocity in meters per second.
     * @param omegaInRadiansPerSecond Angular velocity in radians per second.
     */
    public ChassisSpeeds(double vxInMetersPerSecond, double vyInMetersPerSecond,
            double omegaInRadiansPerSecond) {
        this.vxInMetersPerSecond = vxInMetersPerSecond;
        this.vyInMetersPerSecond = vyInMetersPerSecond;
        this.omegaInRadiansPerSecond = omegaInRadiansPerSecond;
    }

    /**
     * Converts a user provided field-relative set of speeds into a robot-relative
     * ChassisSpeeds object.
     *
     * @param vxInMetersPerSecond     The component of speed in the x direction
     *                                relative to the field.
     *                                Positive x is away from your alliance wall.
     * @param vyInMetersPerSecond     The component of speed in the y direction
     *                                relative to the field.
     *                                Positive y is to your left when standing
     *                                behind the alliance wall.
     * @param omegaInRadiansPerSecond The angular rate of the robot.
     * @param robotAngle              The angle of the robot as measured by a
     *                                gyroscope. The robot's
     *                                angle is considered to be zero when it is
     *                                facing directly away
     *                                from your alliance station wall. Remember that
     *                                this should
     *                                be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the robot's frame of
     *         reference.
     */
    public static ChassisSpeeds fromFieldRelativeSpeeds(
            double vxInMetersPerSecond,
            double vyInMetersPerSecond,
            double omegaInRadiansPerSecond,
            Rotation2d robotAngle) {
        return new ChassisSpeeds(
                vxInMetersPerSecond * robotAngle.cos() + vyInMetersPerSecond * robotAngle.sin(),
                -vxInMetersPerSecond * robotAngle.sin() + vyInMetersPerSecond * robotAngle.cos(),
                omegaInRadiansPerSecond);
    }

    @Override
    public String toString() {
        return String.format("ChassisSpeeds(Vx: %.2f m/s, Vy: %.2f m/s, Omega: %.2f rad/s (%.2f degrees/s)",
                vxInMetersPerSecond, vyInMetersPerSecond, omegaInRadiansPerSecond,
                Math.toDegrees(omegaInRadiansPerSecond));
    }
}
