package libraries.cyberlib.paths;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;
import libraries.cyberlib.spline.QuinticHermiteSpline3D;
import libraries.cyberlib.spline.Spline3D;

import java.util.ArrayList;
import java.util.List;

public class PathBuilder {
    private final List<PathSegment> segmentList = new ArrayList<>();
    private PoseWithCurvatureAndOrientation lastState;
    private double length;

    public PathBuilder(Translation2d initialPosition, Rotation2d initialHeading, Rotation2d initialOrientation) {
        lastState = new PoseWithCurvatureAndOrientation(
                new Pose2d(initialPosition, initialHeading), 0.0, 0.0, initialOrientation.getRadians(), 0.0);
    }

    private void addSegment(PathSegment segment) {
        segmentList.add(segment);
        length += segment.getLength();
        lastState = segment.getEnd();
    }

    private void addSpline(Spline3D spline) {
        addSegment(new Spline3DSegment(spline));
    }

    public Path build() {
        return new Path(segmentList.toArray(new PathSegment[0]));
    }

    public PathBuilder arcTo(Translation2d position, Translation2d center) {
        addSegment(new ArcSegment(lastState.poseMeters.getTranslation(), position, center,
                Rotation2d.fromDegrees(Math.toDegrees(lastState.orientation))));
        return this;
    }

    public PathBuilder arcTo(Translation2d position, Translation2d center,  Rotation2d rotation) {
        addSegment(new ArcSegment(lastState.poseMeters.getTranslation(), position, center,
                Rotation2d.fromDegrees(Math.toDegrees(lastState.orientation)), rotation));
        return this;
    }

    public PathBuilder lineTo(Translation2d position) {
        addSegment(new LineSegment(lastState.poseMeters.getTranslation(),
                Rotation2d.fromDegrees(Math.toDegrees(lastState.orientation)), position));
        return this;
    }

    public PathBuilder lineTo(Translation2d position, Rotation2d rotation) {
        addSegment(new LineSegment(lastState.poseMeters.getTranslation(),
                Rotation2d.fromDegrees(Math.toDegrees(lastState.orientation)), position, rotation));
        return this;
    }

    public PathBuilder lineTo(Translation2d position, Translation2d target) {
        addSegment(new LineSegment(lastState.poseMeters.getTranslation(),
                Rotation2d.fromDegrees(Math.toDegrees(lastState.orientation)), position, target));
        return this;
    }

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
}
