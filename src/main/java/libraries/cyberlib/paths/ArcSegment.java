package libraries.cyberlib.paths;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;
import libraries.cyberlib.utils.Angles;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;


/**
 * Represents an arc defined between two points on a curve and a center point to define the radius.
 */
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
        return ArcParametitizer.parameterize(this);
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

    public static final class ArcParametitizer {
        private static final double kMaxDx = Units.inchesToMeters(2.0);
        private static final double kMaxDy = Units.inchesToMeters(0.05);
        private static final double kMaxDtheta = 0.0872 / 2.0;

        /**
         * A malformed spline does not actually explode the LIFO stack size. Instead, the stack size
         * stays at a relatively small number (e.g. 30) and never decreases. Because of this, we must
         * count iterations. Even long, complex paths don't usually go over 300 iterations, so hitting
         * this maximum should definitely indicate something has gone wrong.
         */
        private static final int kMaxIterations = 5000;

        /**
         * Private constructor because this is a utility class.
         */
        public static List<PoseWithCurvatureAndOrientation> parameterize(PathSegment segment) {
            return parameterize(segment, 0.0, 1.0);
        }

        public static List<PoseWithCurvatureAndOrientation> parameterize(PathSegment segment, double t0, double t1) throws MalformedSplineException {
            var points = new ArrayList<PoseWithCurvatureAndOrientation>();

            // The parameterization does not add the initial point. Let's add that.
            points.add(segment.getPoint(t0));

            // We use an "explicit stack" to simulate recursion, instead of a recursive function call
            // This give us greater control, instead of a stack overflow
            var stack = new ArrayDeque<StackContents>();
            stack.push(new StackContents(t0, t1));
            ArcParametitizer.StackContents current;
            PoseWithCurvatureAndOrientation start;
            PoseWithCurvatureAndOrientation end;
            int iterations = 0;

            while (!stack.isEmpty()) {
                current = stack.removeFirst();
                start = segment.getPoint(current.t0);
                end = segment.getPoint(current.t1);

                final var twist = start.poseMeters.log(end.poseMeters);
                if (Math.abs(twist.dy) > kMaxDy
                        || Math.abs(twist.dx) > kMaxDx
                        || Math.abs(twist.dtheta) > kMaxDtheta) {
                    stack.addFirst(new StackContents((current.t0 + current.t1) / 2, current.t1));
                    stack.addFirst(new StackContents(current.t0, (current.t0 + current.t1) / 2));
                } else {
                    points.add(segment.getPoint(current.t1));
                }

                iterations++;
                if (iterations >= kMaxIterations) {
                    throw new MalformedSplineException(
                            "Could not parameterize a malformed segment. "
                                    + "If this is a spline, you probably had two or more adjacent waypoints that were very close "
                                    + "together with headings in opposing directions."
                    );
                }
            }

            return points;
        }

        public static class MalformedSplineException extends RuntimeException {
            /**
             * Create a new exception with the given message.
             *
             * @param message the message to pass with the exception
             */
            private MalformedSplineException(String message) {
                super(message);
            }
        }

        @SuppressWarnings("MemberName")
        private static class StackContents {
            final double t1;
            final double t0;

            StackContents(double t0, double t1) {
                this.t0 = t0;
                this.t1 = t1;
            }
        }
    }
}