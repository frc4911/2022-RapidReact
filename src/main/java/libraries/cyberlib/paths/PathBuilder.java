package libraries.cyberlib.paths;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;
import libraries.cyberlib.spline.QuinticHermiteSpline3D;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a way to build simple and complex paths composed of different path segments.
 */
public class PathBuilder {
    private final List<PathSegment> segmentList = new ArrayList<>();
    private PoseWithCurvatureAndOrientation lastState;
    private double length;

    /**
     * Creates a PathBuilder instance with an initial position and heading.
     *
     * @param initialPosition The initial position
     * @param initialHeading The initial heading
     */
    public PathBuilder(Translation2d initialPosition, Rotation2d initialHeading) {
        lastState = new PoseWithCurvatureAndOrientation(
                new Pose2d(initialPosition, initialHeading),
                0.0,
                0.0,
                initialHeading.getRadians(),
                0.0);
    }

    private PathBuilder() {
    };

    private void addSegment(PathSegment segment) {
        segmentList.add(segment);
        length += segment.getLength();
        lastState = segment.getEnd();
    }

    public Path build() {
        return new Path(segmentList.toArray(new PathSegment[0]));
    }

    /**
     * Adds an {@link ArcSegment} to the path.
     * <p>The initial position is the position of the last segment.</p>
     * <p>The initial heading and final heading is the heading at the end of the last segment.</p>
     *
     * @param position The end position of the arc
     * @param center The center of the arc
     * @return An instance of the PathBuilder object.
     */
    public PathBuilder arcTo(Translation2d position, Translation2d center) {
        addSegment(new ArcSegment(lastState.poseMeters.getTranslation(), position, center,
                Rotation2d.fromDegrees(Math.toDegrees(lastState.orientation))));
        return this;
    }

    /**
     * Adds an {@link ArcSegment} to the path that rotates to a new heading.
     * <p>The initial position is the position of the last segment.</p>
     * <p>The initial heading is the heading at the end of the last segment.</p>
     *
     * @param position The end position of the arc
     * @param center The center of the arc
     * @param rotation The final heading at the end of the arc
     * @return An instance of the PathBuilder object.
     */
    public PathBuilder arcTo(Translation2d position, Translation2d center,  Rotation2d rotation) {
        addSegment(new ArcSegment(lastState.poseMeters.getTranslation(), position, center,
                Rotation2d.fromDegrees(Math.toDegrees(lastState.orientation)), rotation));
        return this;
    }

    /**
     * Adds an {@link LineSegment} to the path.
     * <p>The initial position is the position of the last segment.</p>
     * <p>The initial heading is the heading at the end of the last segment.</p>
     *
     * @param position The end position of the line segment
     * @return An instance of the PathBuilder object.
     */
    public PathBuilder lineTo(Translation2d position) {
        addSegment(new LineSegment(lastState.poseMeters.getTranslation(),
                Rotation2d.fromDegrees(Math.toDegrees(lastState.orientation)), position));
        return this;
    }

    /**
     * Adds an {@link LineSegment} to the path.
     * <p>The initial position is the position of the last segment.</p>
     * <p>The initial heading is the heading at the end of the last segment.</p>
     *
     * @param position The end position of the line segment
     * @param rotation The final heading at the end of the arc
     * @return An instance of the PathBuilder object.
     */
    public PathBuilder lineTo(Translation2d position, Rotation2d rotation) {
        addSegment(new LineSegment(lastState.poseMeters.getTranslation(),
                Rotation2d.fromDegrees(Math.toDegrees(lastState.orientation)), position, rotation));
        return this;
    }

    /**
     * Adds an {@link LineSegment} to the path.
     * <p>The initial position is the position of the last segment.</p>
     * <p>The initial heading is the heading at the end of the last segment.</p>
     * <p>The final  heading is the heading toward the target..</p>
     *
     * @param position The end position of the line segment
     * @param target A fixed position the heading is to point through the line segment
     * @return An instance of the PathBuilder object.
     */
    public PathBuilder lineTo(Translation2d position, Translation2d target) {
        addSegment(new LineSegment(lastState.poseMeters.getTranslation(),
                Rotation2d.fromDegrees(Math.toDegrees(lastState.orientation)), position, target));
        return this;
    }

    /**
     * Adds an {@link Spline3DSegment} to the path.
     * <p>The initial position is the position of the last segment.</p>
     * <p>The initial heading and final heading is the heading at the end of the last segment.</p>
     *
     * @param position The end position of the line segment
     * @return An instance of the PathBuilder object.
     */
    public PathBuilder splineTo(Translation2d position) {
        var positionAngle = new Rotation2d(position.getX(), position.getY());

        var initialX = new double[] {
                lastState.poseMeters.getTranslation().getX(),
                lastState.poseMeters.getRotation().getCos(),
                0.0};

        var finalX = new double[] {
                position.getX(),
                positionAngle.getCos(),
                0.0 };

        var initialY = new double[] {
                lastState.poseMeters.getTranslation().getY(),
                lastState.poseMeters.getRotation().getSin(),
                0.0};

        var finalY = new double[] {
                position.getY(),
                positionAngle.getSin(),
                0.0};

        var initialZ = new double[] {
                lastState.orientation,
                lastState.angularVelocity,
                lastState.angularAcceleration};

        // Don't change orientation
        var finalZ = new double[] {
                lastState.orientation,
                0.0,
                0.0};

        var spline = new QuinticHermiteSpline3D(initialX, finalX, initialY, finalY, initialZ, finalZ);

        addSegment(new Spline3DSegment(spline));
        return this;
    }

    /**
     * Adds an {@link Spline3DSegment} to the path.
     * <p>The initial position is the position of the last segment.</p>
     * <p>The initial heading and final heading is the heading at the end of the last segment.</p>
     *
     * @param position The end position of the spline segment
     * @param rotation The final heading at the end of the spline
     * @return An instance of the PathBuilder object.
     */
    public PathBuilder splineTo(Translation2d position, Rotation2d rotation) {
        var positionAngle = new Rotation2d(position.getX(), position.getY());

        var initialX = new double[] {
                lastState.poseMeters.getTranslation().getX(),
                lastState.poseMeters.getRotation().getCos(),
                0.0};

        var finalX = new double[] {
                position.getX(),
                positionAngle.getCos(),
                0.0 };

        var initialY = new double[] {
                lastState.poseMeters.getTranslation().getY(),
                lastState.poseMeters.getRotation().getSin(),
                0.0};

        var finalY = new double[] {
                position.getY(),
                positionAngle.getSin(),
                0.0};

        var initialZ = new double[] {
                lastState.orientation,
                lastState.angularVelocity,
                lastState.angularAcceleration};

        // Don't change orientation
        var finalZ = new double[] {
                rotation.getRadians(),
                0.0,
                0.0};

        var spline = new QuinticHermiteSpline3D(initialX, finalX, initialY, finalY, initialZ, finalZ);

        addSegment(new Spline3DSegment(spline));
        return this;
    }
}
