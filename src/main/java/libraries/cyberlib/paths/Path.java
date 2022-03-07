package libraries.cyberlib.paths;

import edu.wpi.first.math.util.Units;
import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

public class Path {

    public final PathSegment[] segments;

    public Path(PathSegment[] segments) {
        this.segments = segments;
    }

    public PathSegment[] getSegments() {
        return segments;
    }

    public List<PoseWithCurvatureAndOrientation> parameterize() {
        List<PoseWithCurvatureAndOrientation> points = new ArrayList<>();
        for (var segment : segments) {
            points.addAll(segment.parameterize());
        }

        return points;
    }

    public List<List<PoseWithCurvatureAndOrientation>> parameterizeAsList() {
        List<List<PoseWithCurvatureAndOrientation>> list = new ArrayList<>();
        List<PoseWithCurvatureAndOrientation> points = new ArrayList<>();
        for (var segment : segments) {
            points.addAll(segment.parameterize());
            list.add(points);
        }

        return list;
    }

    public static class SegmentParametitizer {
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
            SegmentParametitizer.StackContents current;
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
