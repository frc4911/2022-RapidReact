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

import static libraries.cyberlib.utils.Angles.normalizeAngle;

/**
 * Represents a straight line path segment between a start and an end position.
 */
public class LineSegment extends PathSegment {

    private final Translation2d start;
    private final Translation2d end;
    private final double length;
    private Rotation2d startOrientation;
    private Rotation2d endOrientation;
    private final boolean isTargetEnabled = false;
    private final Rotation2d lineAngle;
    private Translation2d target;
    double kRamp = 0.25;

    public LineSegment(Translation2d start, Rotation2d startOrientation, Translation2d end) {
        this.start = start;
        this.end = end;
        this.startOrientation = startOrientation;
        this.lineAngle = Rotation2d.fromDegrees(
                Math.toDegrees(Angles.getLineAngle(
                        List.of(start.getX(), start.getY()),
                        List.of(end.getX(), end.getY()))));
        var delta = end.minus(start);
        this.length = delta.getNorm();
        this.endOrientation = Rotation2d.fromDegrees(startOrientation.getDegrees());
        this.target = new Translation2d();
    }

    /**
     * Create a path segment where robot heading will change between start and end orientations.
     *
     * @param start            The start position of the segment.
     * @param end              The end position of the segment.
     * @param startOrientation The start orientation of the robot at the start position of the segment.
     * @param endOrientation   The end orientation of the robot at the end position of the segment.
     */
    public LineSegment(Translation2d start, Rotation2d startOrientation, Translation2d end, Rotation2d endOrientation) {
        this(start, startOrientation, end);
        var minimumAngle = Angles.shortest_angular_distance(
                startOrientation.getRadians(), endOrientation.getRadians());
        this.endOrientation = startOrientation.plus(new Rotation2d(minimumAngle));
    }

    /**
     * Create a path segment where robot heading will point to target position.
     *
     * @param start  The start position of the segment.
     * @param end    The end position of the segment.
     * @param target The target point for the heading through the segment.
     */
    public LineSegment(Translation2d start, Rotation2d startOrientation, Translation2d end, Translation2d target) {
        this(start, startOrientation, end);
        this.target = target;
//        this.startOrientation = new Rotation2d(
//                normalizeAngle(
//                        Math.atan2(target.getY() - start.getY(), target.getX() - start.getX())));
        this.endOrientation = new Rotation2d(
                normalizeAngle(
                        Math.atan2(target.getY() - end.getY(), target.getX() - end.getX())));
        boolean isTargetEnabled = true;
    }

    @Override
    public PoseWithCurvatureAndOrientation getPoint(double t) {
//        var t = distance / getLength();
        Rotation2d orientation = startOrientation.interpolate(endOrientation, t);
        if (isTargetEnabled && t < kRamp) {
            orientation = Rotation2d.fromDegrees(
                    Math.toDegrees(getStart().orientation)).interpolate(orientation, t / kRamp);
        }

        return new PoseWithCurvatureAndOrientation(
                new Pose2d(
                        start.interpolate(end, t),
                        lineAngle),
                0.0,
                orientation.getRadians(),
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
        return LineParameterizer.parameterize(this);
    }

    public static final class LineParameterizer {
//    private static final double kMaxDx = 0.127; // 2.0;
//    private static final double kMaxDy = 0.00127; // 0.05;
//    private static final double kMaxDtheta = 0.0872;

        private static final double kMaxDx = Units.inchesToMeters(1.0);
        private static final double kMaxDy = Units.inchesToMeters(1.0);
        private static final double kMaxDtheta = 0.0872 / 2.0;

        /**
         * A malformed spline does not actually explode the LIFO stack size. Instead, the stack size
         * stays at a relatively small number (e.g. 30) and never decreases. Because of this, we must
         * count iterations. Even long, complex paths don't usually go over 300 iterations, so hitting
         * this maximum should definitely indicate something has gone wrong.
         */
        private static final int kMaxIterations = 5000;

        @SuppressWarnings("MemberName")
        private static class StackContents {
            final double t1;
            final double t0;

            StackContents(double t0, double t1) {
                this.t0 = t0;
                this.t1 = t1;
            }
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

        /**
         * Private constructor because this is a utility class.
         */
        private LineParameterizer() {
        }

        /**
         * Parameterizes the spline. This method breaks up the spline into various
         * arcs until their dx, dy, and dtheta are within specific tolerances.
         *
         * @param line The line to parameterize.
         * @return A list of poses and curvatures that represents various points on the spline.
         * @throws MalformedSplineException When the spline is malformed (e.g. has close adjacent points
         *                                                                                  with approximately opposing headings)
         */
        public static List<PoseWithCurvatureAndOrientation> parameterize(LineSegment line) {
            return parameterize(line, 0.0, 1.0);
        }

        /**
         * Parameterizes the spline. This method breaks up the spline into various
         * arcs until their dx, dy, and dtheta are within specific tolerances.
         *
         * @param line The spline to parameterize.
         * @param t0     Starting internal spline parameter. It is recommended to use 0.0.
         * @param t1     Ending internal spline parameter. It is recommended to use 1.0.
         * @return A list of poses and curvatures that represents various points on the spline.
         * @throws MalformedSplineException When the spline is malformed (e.g. has close adjacent points
         *                                                                                  with approximately opposing headings)
         */
        @SuppressWarnings("PMD.AvoidInstantiatingObjectsInLoops")
        public static List<PoseWithCurvatureAndOrientation> parameterize(LineSegment line, double t0, double t1) throws libraries.cyberlib.spline.Spline3DParameterizer.MalformedSplineException {
            var linePoints = new ArrayList<PoseWithCurvatureAndOrientation>();

            // The parameterization does not add the initial point. Let's add that.
            linePoints.add(line.getPoint(t0));

            // We use an "explicit stack" to simulate recursion, instead of a recursive function call
            // This give us greater control, instead of a stack overflow
            var stack = new ArrayDeque<StackContents>();
            stack.push(new StackContents(t0, t1));

            StackContents current;
            PoseWithCurvatureAndOrientation start;
            PoseWithCurvatureAndOrientation end;
            int iterations = 0;

            while (!stack.isEmpty()) {
                current = stack.removeFirst();
                start = line.getPoint(current.t0);
                end = line.getPoint(current.t1);

                final var twist = start.poseMeters.log(end.poseMeters);
                if (Math.abs(twist.dy) > kMaxDy
                        || Math.abs(twist.dx) > kMaxDx
                        /* || Math.abs(twist.dtheta) > kMaxDtheta */) {
                    stack.addFirst(new StackContents((current.t0 + current.t1) / 2, current.t1));
                    stack.addFirst(new StackContents(current.t0, (current.t0 + current.t1) / 2));
                } else {
                    linePoints.add(line.getPoint(current.t1));
                }

                iterations++;
                if (iterations >= kMaxIterations) {
                    throw new MalformedSplineException(
                            "Could not parameterize a malformed spline. "
                                    + "This means that you probably had two or more adjacent waypoints that were very close "
                                    + "together with headings in opposing directions."
                    );
                }
            }

            return linePoints;
        }
    }
}