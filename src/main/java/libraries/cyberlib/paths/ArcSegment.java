package libraries.cyberlib.paths;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;
import libraries.cyberlib.utils.Angles;

import java.util.List;

public class ArcSegment extends PathSegment {
    private final Translation2d center;
    private final Translation2d deltaStart;
    private final Translation2d deltaEnd;
    private final boolean clockwise;
    private final Rotation2d arcAngle;
    private final double curvature;
    private  final double length;
    private Rotation2d startOrientation;
    private Rotation2d endOrientation;

    public ArcSegment(Translation2d start, Translation2d end, Translation2d center,
                      Rotation2d startOrientation) {
        this.center = center;
        this.deltaStart = start.minus(center);
        this.deltaEnd = end.minus(center);
        this.startOrientation = startOrientation;
        this.endOrientation = Rotation2d.fromDegrees(startOrientation.getDegrees());

        var cross = deltaStart.getX() * deltaEnd.getY() - deltaStart.getY() * deltaEnd.getX();
        clockwise = cross <= 0.0;

        var r1 = new Rotation2d(deltaStart.getX(), deltaStart.getY());
        var r2 = new Rotation2d(deltaEnd.getX(), deltaEnd.getY());
        this.arcAngle = Rotation2d.fromDegrees(
                Math.toDegrees(Angles.shortest_angular_distance(r1.getRadians(), r2.getRadians())));

        this.curvature = 1.0 / deltaStart.getNorm();
        this.length = deltaStart.getNorm() * arcAngle.getRadians();
    }

    public ArcSegment(Translation2d start, Translation2d end, Translation2d center,
                      Rotation2d startOrientation, Rotation2d endOrientation) {
        this(start, end, center, startOrientation);
        this.endOrientation = endOrientation;
    }

    @Override
    public PoseWithCurvatureAndOrientation getPoint(double t) {
        double sampleAngle = arcAngle.getDegrees() * t;
        // TODO: Use cross product instead of just adding 90deg when calculating heading
        Translation2d sampleHeading = deltaStart.rotateBy(Rotation2d.fromDegrees(sampleAngle + (clockwise ? -1.0 : 1.0) * 90));
//        Translation2d sampleHeading = deltaStart.rotateBy(Rotation2d.fromDegrees(sampleAngle));
        Rotation2d newHeading = new Rotation2d(sampleHeading.getX(), sampleHeading.getY());
        return new PoseWithCurvatureAndOrientation(
                new Pose2d(center.plus(deltaStart.rotateBy(Rotation2d.fromDegrees(sampleAngle))), newHeading),
                curvature,
                startOrientation.interpolate(endOrientation, t).getRadians(),
                0.0,
                0.0);
    }

    @Override
    public double getLength() {
        return length;
    }

    @Override
    public List<PoseWithCurvatureAndOrientation> parameterize() {
        // Use default parameterizer.
        return Path.SegmentParametitizer.parameterize(this);
    }

//    public double dot(Translation2d me, Translation2d other) {
//        return me.getX() * other.getY() + me.getY() * other.getX();
//    }
//
//    public double cross(Translation2d me, Translation2d other) {
//        return me.getX() * other.getY() - me.getY() * other.getX();
//    }
//
//    Rotation2d getAngleBetween(Translation2d a, Translation2d b) {
//        double cos = dot(a, b) / (a.getNorm() * b.getNorm());
//        if (Double.isNaN(cos)) {
//            return new Rotation2d();
//        }
//
//        return Rotation2d.fromDegrees(Math.toDegrees(Math.acos(Util.limit(cos, -1.0, 1.0))));
//    }
}