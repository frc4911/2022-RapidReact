package libraries.cyberlib.spline;

import edu.wpi.first.math.geometry.Pose2d;

import java.text.DecimalFormat;

/**
 * Represents a pose, a curvature, and an orientation.
 */
@SuppressWarnings("MemberName")
public class PoseWithCurvatureAndOrientation {
    // Represents the pose.
    public Pose2d poseMeters;

    // Represents the curvature.
    public double curvatureRadPerMeter;

    // Represents the orientation.
    public double orientation;

    // Represents the angular velocity.
    public double angularVelocity;

    // Represents the angular acceleration.
    public double angularAcceleration;

    /**
     * Constructs a PoseWithCurvature.
     *
     * @param poseMeters           The pose.
     * @param curvatureRadPerMeter The curvature.
     * @param orientation          The orientation in radians.
     * @param angularVelocity      The orientation in radians per second.
     */
    public PoseWithCurvatureAndOrientation(
            Pose2d poseMeters,
            double curvatureRadPerMeter,
            double orientation,
            double angularVelocity,
            double angularAcceleration) {
        this.poseMeters = poseMeters;
        this.curvatureRadPerMeter = curvatureRadPerMeter;
        this.orientation = orientation;
        this.angularVelocity = angularVelocity;
        this.angularAcceleration = angularAcceleration;
    }

    /**
     * Constructs a PoseWithCurvature with default values.
     */
    public PoseWithCurvatureAndOrientation() {
        this(new Pose2d(), 0.0, 0.0, 0.0, 0.0);
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return poseMeters.toString() +
                ", curvature: " + fmt.format(curvatureRadPerMeter) +
                ", orientation:(Rads: " + fmt.format(orientation) +", Deg: " + fmt.format(Math.toDegrees(orientation)) + ")" +
                ", angularVelocity " +  fmt.format(angularVelocity) +
                ", angularAcceleration " + fmt.format(angularAcceleration);
    }

}
