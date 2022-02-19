package libraries.cyberlib.spline;

import edu.wpi.first.math.util.Units;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

public final class Spline3DParameterizer {
//    private static final double kMaxDx = 0.127; // 2.0;
//    private static final double kMaxDy = 0.00127; // 0.05;
//    private static final double kMaxDtheta = 0.0872;

    private static final double kMaxDx = Units.inchesToMeters(2.0);
    private static final double kMaxDy = Units.inchesToMeters(0.05);
    private static final double kMaxDtheta = 0.0872/2.0;

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
    private Spline3DParameterizer() {
    }

    /**
     * Parameterizes the spline. This method breaks up the spline into various
     * arcs until their dx, dy, and dtheta are within specific tolerances.
     *
     * @param spline The spline to parameterize.
     * @return A list of poses and curvatures that represents various points on the spline.
     * @throws MalformedSplineException When the spline is malformed (e.g. has close adjacent points
     *                                  with approximately opposing headings)
     */
    public static List<PoseWithCurvatureAndOrientation> parameterize(Spline3D spline) {
        return parameterize(spline, 0.0, 1.0);
    }

    /**
     * Parameterizes the spline. This method breaks up the spline into various
     * arcs until their dx, dy, and dtheta are within specific tolerances.
     *
     * @param spline The spline to parameterize.
     * @param t0     Starting internal spline parameter. It is recommended to use 0.0.
     * @param t1     Ending internal spline parameter. It is recommended to use 1.0.
     * @return       A list of poses and curvatures that represents various points on the spline.
     * @throws MalformedSplineException When the spline is malformed (e.g. has close adjacent points
     *                                  with approximately opposing headings)
     */
    @SuppressWarnings("PMD.AvoidInstantiatingObjectsInLoops")
    public static List<PoseWithCurvatureAndOrientation> parameterize(Spline3D spline, double t0, double t1) throws MalformedSplineException {
        var splinePoints = new ArrayList<PoseWithCurvatureAndOrientation>();

        // The parameterization does not add the initial point. Let's add that.
        splinePoints.add(spline.getPoint(t0));

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
            start = spline.getPoint(current.t0);
            end = spline.getPoint(current.t1);

            final var twist = start.poseMeters.log(end.poseMeters);
            if (Math.abs(twist.dy) > kMaxDy
                    || Math.abs(twist.dx) > kMaxDx
                    || Math.abs(twist.dtheta) > kMaxDtheta) {
                stack.addFirst(new StackContents((current.t0 + current.t1) / 2, current.t1));
                stack.addFirst(new StackContents(current.t0, (current.t0 + current.t1) / 2));
            } else {
                splinePoints.add(spline.getPoint(current.t1));
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

        return splinePoints;
    }
}
