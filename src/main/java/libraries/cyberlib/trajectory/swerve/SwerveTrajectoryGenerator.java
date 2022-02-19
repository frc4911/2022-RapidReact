package libraries.cyberlib.trajectory.swerve;

import libraries.cheesylib.geometry.Translation2d;
import libraries.cyberlib.spline.QuinticHermiteSpline3D;
import libraries.cyberlib.spline.Spline3DHelper;
import libraries.cyberlib.trajectory.Trajectory;
import libraries.cyberlib.trajectory.TrajectoryConfig;
import libraries.cyberlib.trajectory.TrajectoryGenerator;
import libraries.cyberlib.utils.Angles;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class SwerveTrajectoryGenerator {

    /**
     * Represents a trajectory point.
     * Index 0 = x
     * Index 1 = y
     * Index 2 = theta (or the orientation of the 2-D robot)
     */
    public static class TrajectoryPoint {
        public final List<Double> positions = new ArrayList<>();
        public final List<Double> velocities = new ArrayList<>();
        public final List<Double> accelerations = new ArrayList<>();
    }

    public static Trajectory generateTrajectory(List<Translation2d> wayPoints, TrajectoryConfig config) {
        Trajectory trajectory = new Trajectory();
        return trajectory;
    }

    /**
     * Creates a trajectory as a list of QuinticHermiteSplines
     *
     * @param wayPoints        List of waypoints
     * @param startOrientation The start orientation (in radians)
     * @param endOrientation   The finish orientation (in radians)
     * @param config           A TrajectoryConfiguration
     * @return A list of QuinticHermiteSplines representing the path
     */
    public static Trajectory create_trajectory(
            List<Translation2d> wayPoints,
            double startOrientation,
            double endOrientation,
            TrajectoryConfig config,
            List<QuinticHermiteSpline3D> splines) {

        System.out.println("wayPoints");
        for (var wayPoint : wayPoints) {
            System.out.println(wayPoint.toString());
        }

        List<TrajectoryPoint> points = createTrajectoryPoints(wayPoints);

        // Create an optimization parameter set per waypoint
        List<OptimizationParameters> optParams = new ArrayList<>();
        for (var i = 0; i < points.size(); i++) {
            optParams.add(new OptimizationParameters());
        }

      // var d_splines = generateSpline(points, startOrientation, endOrientation, optParams);

        Trajectory trajectory = RProp(points, startOrientation, endOrientation, config,
                0.05,
                0.0075,
                0.05,
                splines);

        System.out.println("Splines");
        for (QuinticHermiteSpline3D spline : splines) {
            System.out.println(spline.toString());
        }

        System.out.println();
        System.out.println();

        System.out.println("Trajectory");
        System.out.println(trajectory.toString());

        return trajectory;
    }

    /**
     * Converts a list of 2-d waypoints into a list of TrajectoryPoint
     *
     * @param wayPoints A list of 2-d waypoints
     * @return A list of corresponding TrajectoryPoints.
     */
    public static List<TrajectoryPoint> createTrajectoryPoints(List<Translation2d> wayPoints) {
        // TODO:  If we move the inner waypoints, then the lengths and angles will be off slightly.
        List<TrajectoryPoint> points = new ArrayList<>();

        for (Translation2d point : wayPoints) {
            var trajectoryPoint = new TrajectoryPoint();
            trajectoryPoint.positions.add(point.x());
            trajectoryPoint.positions.add(point.y());
            trajectoryPoint.positions.add(0.0);
            points.add(trajectoryPoint);
        }

        var angle = Angles.getLineAngle(points.get(0).positions, points.get(1).positions);
        var length = Math.hypot(
                points.get(1).positions.get(0) - points.get(0).positions.get(0),
                points.get(1).positions.get(1) - points.get(0).positions.get(1));

        points.get(0).velocities.add(length * Math.cos(angle));
        points.get(0).velocities.add(length * Math.sin(angle));
        points.get(0).velocities.add(0.0);
        points.get(0).accelerations.add(0.0);
        points.get(0).accelerations.add(0.0);
        points.get(0).accelerations.add(0.0);

        var last = points.size() - 1;

        angle = Angles.getLineAngle(points.get(last).positions, points.get(last).positions);
        length = Math.hypot(
                points.get(last).positions.get(0) - points.get(last - 1).positions.get(0),
                points.get(last).positions.get(1) - points.get(last - 1).positions.get(1));

        points.get(last).velocities.add(length * Math.cos(angle));
        points.get(last).velocities.add(length * Math.sin(angle));
        points.get(last).velocities.add(0.0);
        points.get(last).accelerations.add(0.0);
        points.get(last).accelerations.add(0.0);
        points.get(last).accelerations.add(0.0);

        return points;
    }

    /**
     * Generate quintic spline segments given a set of waypoints and optimization parameters.
     *
     * @param points    The waypoints.
     * @param optParams The current optimization parameters to apply when generating splines.
     * @return Updated trajectory with new spline segments appended.
     */
    public static List<QuinticHermiteSpline3D> generateSpline(
            List<TrajectoryPoint> points,
            double startOrientation,
            double endOrientation,
            List<OptimizationParameters> optParams) {

        List<Segment> segments = new ArrayList<>();
        double total_length = 0.0;

        // Auto generate translational velocities and accelerations for splines
        double prevAngle = Angles.getLineAngle(points.get(0).positions, points.get(1).positions);
        double prevLength = Math.hypot(
                points.get(1).positions.get(0) - points.get(0).positions.get(0),
                points.get(1).positions.get(1) - points.get(0).positions.get(1));

        total_length += prevLength;
        Segment segment = new Segment();
        segment.angle = prevAngle;
        segment.segment_length = prevLength;
        segment.initial_orientation = prevAngle;
        segments.add(segment);

        for (int i = 1;  i < points.size() - 1; i++) {
            List<Double> mi = points.get(i).positions;
            List<Double> mip1 = points.get(i + 1).positions;

            double currAngle = Angles.getLineAngle(mi, mip1);
            double deltaAngle = currAngle - prevAngle;

            // Normalize the angle
            if (deltaAngle < -Math.PI) {
                deltaAngle += 2.0 * Math.PI;
            } else if (deltaAngle > Math.PI) {
                deltaAngle -= 2.0 * Math.PI;
            }

            // Calculate tangent angle
            double angle = prevAngle + deltaAngle / 2;
            if (angle > Math.PI) {
                angle -= 2.0 * Math.PI;
            }

            // The elongation factor controls the velocity at the waypoint and affects the curvature of the path.
            // Greater than 1 ==> curvier path with higher speeds.
            // Less than 1 ==> tighter turns to stay closer to straight paths.

            double currLength = Math.hypot(mip1.get(0) - mi.get(0), mip1.get(1) - mi.get(1));
            total_length += currLength;

//            double length = optParams.get(i).elongation * 0.5 * Math.min(currLength, prevLength);
            double length = optParams.get(i).elongation * Math.min(currLength, prevLength);

//            System.out.println(
//                    String.format("prevAngle=%.3f, prevLength=%.3f, ", Math.toDegrees(prevAngle), prevLength) +
//                    String.format("prevAngle=%.3f, prevLength=%.3f, ", Math.toDegrees(currAngle), currLength) +
//                    String.format("currAngle - deltaAngle=%.3f, ", Math.toDegrees(currAngle - deltaAngle)) +
//                    String.format("normalized=%.3f, ", Math.toDegrees(Util.normalize_angle_positive(currAngle - deltaAngle))) +
//                    String.format("angle=%.3f, length=%.3f", Math.toDegrees(angle), length)
//            );

            // Update velocities (first derivative)
            if (points.get(i).velocities.size() == 0) {
                points.get(i).velocities.add(length * Math.cos(angle));
            }
            if (points.get(i).velocities.size() == 1) {
                points.get(i).velocities.add(length * Math.sin(angle));
            }
            if (points.get(i).velocities.size() == 2) {
                points.get(i).velocities.add(0.0);
            }

            // Capture segment values
            segment = new Segment();
            segment.segment_length = currLength;
            segment.angle = Angles.normalizeAngle(currAngle);
            segment.initial_orientation = segment.angle;
            segments.add(segment);

            // Update for next step
            prevAngle = currAngle;
            prevLength = currLength;
        }

        // Now estimate the acceleration, or 2nd derivative using a heuristic
        // based on cubic bezier splines.

        // First and last point don't have a prev / next point to connect to,
        // so just grab the 2nd derivative off the point they are adjacent to.
        set_first_last_point_acceleration(
                points.get(1).positions,
                points.get(0).positions,
                points.get(1).velocities,
                points.get(0).velocities,
                points.get(0).accelerations);

        var last = points.size() - 1;
        set_first_last_point_acceleration(
                points.get(last - 1).positions,
                points.get(last).positions,
                points.get(last - 1).velocities,
                points.get(last).velocities,
                points.get(last).accelerations);

        // For interior points, weight the average of the 2nd derivative of each cubic equivalent
        // splines and use them as the acceleration for the to-be-generated quintic spline.
        for (int i = 1; i < points.size() - 1; i++) {
            double Ax  = points.get(i - 1).positions.get(0);
            double Ay  = points.get(i - 1).positions.get(1);
            double tAx = points.get(i - 1).velocities.get(0);
            double tAy = points.get(i - 1).velocities.get(1);

            double Bx  = points.get(i).positions.get(0);
            double By  = points.get(i).positions.get(1);
            double tBx = points.get(i).velocities.get(0);
            double tBy = points.get(i).velocities.get(1);

            double Cx = points.get(i + 1).positions.get(0);
            double Cy = points.get(i + 1).positions.get(1);
            double tCx = points.get(i + 1).velocities.get(0);
            double tCy = points.get(i + 1).velocities.get(1);

            // L2 distance between A and B
            double dab = Math.hypot(Bx - Ax, By - Ay); 
            //L2 distance between B and C
            double dbc = Math.hypot(Cx - Bx, Cy - By);

            // Weighting factors
            double alpha = dbc / (dab + dbc);
            double beta  = dab / (dab + dbc);

            double xaccel = alpha * (6.0 * Ax + 2.0 * tAx + 4.0 * tBx - 6.0 * Bx) +
                            beta * (-6.0 * Bx - 4.0 * tBx - 2.0 * tCx + 6.0 * Cx);
            double yaccel = alpha * (6.0 * Ay + 2.0 * tAy + 4.0 * tBy - 6.0 * By) + 
                            beta * (-6.0 * By - 4.0 * tBy - 2.0 * tCy + 6.0 * Cy);

            // Don't overwrite requested input accelerations
            if (points.get(i).accelerations.size() == 0) {
                points.get(i).accelerations.add(xaccel);  // x
            }
            if (points.get(i).accelerations.size() == 1) {
                points.get(i).accelerations.add(yaccel);  // y
            }
            if (points.get(i).accelerations.size() == 2) {
                points.get(i).accelerations.add(0.0);   // theta
            }

//            System.out.println(
//                    String.format("dab=%.3f, dbc=%.3f, ", dab, dbc) +
//                    String.format("Ax=%.3f, tAx=%.3f, Bx=%.3f, tBx=%.3f, Cx=%.3f, tCx=%.3f, ", Ax, tAx, Bx, tBx, Cx, tCx) +
//                    String.format("Ay=%.3f, tAy=%.3f, By=%.3f, tBy=%.3f, Cy=%.3f, tCy=%.3f, ", Ay, tAy, By, tBy, Cy, tCy)
//            );
        }

        // Generate the splines for each segment
        List<QuinticHermiteSpline3D> splines = new ArrayList<>();
        for (int i = 1; i < points.size(); i++) {
            QuinticHermiteSpline3D curr_spline = new QuinticHermiteSpline3D(
                    new double[] {points.get(i-1).positions.get(0), points.get(i-1).velocities.get(0), points.get(i-1).accelerations.get(0)},
                    new double[] {points.get(i).positions.get(0), points.get(i).velocities.get(0), points.get(i).accelerations.get(0)},
                    new double[] {points.get(i-1).positions.get(1), points.get(i-1).velocities.get(1), points.get(i-1).accelerations.get(1)},
                    new double[] {points.get(i).positions.get(1), points.get(i).velocities.get(1), points.get(i).accelerations.get(1)});
            splines.add(curr_spline);

            // Capture segment values
            segments.get(i - 1).splines.add(curr_spline);
            segments.get(i - 1).spline = Optional.of(curr_spline);
        }

        // --------------------------------------------------------
        // Now for the rotational component
        // First, calculate the path fractions
        double theta_diff = Angles.normalizeAngle(endOrientation - startOrientation);
        double running_total = 0.0;

        if (startOrientation != Math.toRadians(-180)) {
            startOrientation = Angles.normalizeAngle(startOrientation);
        }

        if (endOrientation != Math.toRadians(-180)) {
            endOrientation = Angles.normalizeAngle(endOrientation);
        }

        for (int i = 0; i < segments.size(); i++) {
            segments.get(i).path_fraction = (running_total + (0.5 * segments.get(i).segment_length)) / total_length;
            segments.get(i).minimized_orientation = Angles.normalizeAngle(
                    startOrientation + segments.get(i).path_fraction * theta_diff);
            running_total += segments.get(i).segment_length;

            // Special case when start and end orientation are the same
            if (theta_diff == 0.0) {
                segments.get(i).initial_orientation = startOrientation;
            }
        }

        // Calculate orientations.  The initial path planner assumes the robot's orientation
        // is that of the segment and that the robot  turns in-place at the waypoints.
        for (int i = 0; i < segments.size(); i++) {
            // blend the rotational behavior for segment(i) between initial orientation and minimized orientation,
            segments.get(i).orientation =
                    ((1.0 - optParams.get(i).theta_lambda) * segments.get(i).initial_orientation) +
                    (optParams.get(i).theta_lambda * segments.get(i).minimized_orientation);
            // initialize segment's start and end orientation assuming no rotation
            segments.get(i).splines.get(0).zInitialControlVector[0] = segments.get(i).orientation;
            segments.get(i).splines.get(0).zFinalControlVector[0] = segments.get(i).orientation;
            segments.get(i).rotation_start = optParams.get(i).rs;
            segments.get(i).rotation_end = optParams.get(i).re;
        }

        applyRotation1(segments, startOrientation, endOrientation, optParams);

        // Now apply second derivatives for rotational control points
        for (int i = 1; i < segments.size(); i++) {
            // Set second derivative for inner waypoints segments using same cubic spline heuristic
            var lastSpline = segments.get(i - 1).splines.size() - 1;
            double Ax  = segments.get(i-1).splines.get(lastSpline).zInitialControlVector[0];
            double tAx = segments.get(i-1).splines.get(lastSpline).zInitialControlVector[1];

            double Bx  = segments.get(i).splines.get(0).zInitialControlVector[0];
            double tBx = segments.get(i).splines.get(0).zInitialControlVector[1];

            double Cx  = segments.get(i).splines.get(0).zFinalControlVector[0];
            double tCx = segments.get(i).splines.get(0).zFinalControlVector[1];

            // L2 distance between A and B
            double dab = segments.get(i-1).segment_length;
            // L2 distance between B and C
            double dbc = segments.get(i).segment_length;

            //Weighting factors
            double alpha = dbc / (dab + dbc);
            double beta = dab / (dab + dbc);

            double rot_accel = alpha * (6.0 * Ax + 2.0 * tAx + 4.0 * tBx - 6.0 * Bx) +
                               beta * (-6.0 * Bx - 4.0 * tBx - 2.0 * tCx + 6.0 * Cx);

            if (segments.get(i-1).rotation_start > 0.0) {
                segments.get(i-1).splines.get(segments.get(i-1).splines.size() - 1).zFinalControlVector[2] = rot_accel;
            }

            if (segments.get(i).rotation_end > 0.0) {
                segments.get(i).splines.get(0).zInitialControlVector[2] = rot_accel;
            }
        }

        // Remember, rotational control points span segments
        // Set first waypoint's second derivative only if an ending control point is defined on first segment
        if (segments.get(0).rotation_end > 0.0) {
            segments.get(0).splines.get(0).zInitialControlVector[2] = get_first_last_rotation_control_acceleration(
                    segments.get(0).splines.get(0).zInitialControlVector[0],
                    segments.get(0).splines.get(0).zFinalControlVector[0],
                    segments.get(0).splines.get(0).zInitialControlVector[1],
                    segments.get(0).splines.get(0).zFinalControlVector[1]);
        }

        // Set last waypoint's second derivative only if control point is defined on last segment.
        last = segments.size() - 1;
        var last_spline = segments.get(last).splines.size() - 1;
        if (segments.get(last).rotation_start > 0.0) {
            segments.get(last).splines.get(last_spline).zFinalControlVector[2] = get_first_last_rotation_control_acceleration(
                    segments.get(last).splines.get(last_spline).zInitialControlVector[0],
                    segments.get(last).splines.get(last_spline).zFinalControlVector[0],
                    segments.get(last).splines.get(last_spline).zInitialControlVector[1],
                    segments.get(last).splines.get(last_spline).zFinalControlVector[1]);
        }

        if (segments.get(last).rotation_start != 0.0 &&
                segments.get(last).splines.get(last_spline).xFinalControlVector[2] == 0.0) {
            segments.get(last).splines.get(last_spline).zFinalControlVector[2] = 0.0;
        }

        splines.clear();
        for (Segment s : segments) {
            splines.addAll(s.splines);
        }

        // Recalculate coefficients now the rotational values are set.
        for (QuinticHermiteSpline3D spline : splines) {
            spline.calculateCoefficients();
//            System.out.println(spline.toString());
        }

//        System.out.println();

        return splines;
    }

    private static void applyRotation(
            List<Segment> segments,
            double startOrientation,
            double endOrientation,
            List<OptimizationParameters> optParams) {
        int last;
        // First segment
        // re distributes on the outgoing segment from the waypoint w(i)
        double re = segments.get(0).rotation_end;

        // rs  distributes on the incoming segment to the next waypoint w(i+1)
        double rs = segments.get(0).rotation_start;

        // set start orientation
        segments.get(0).splines.get(0).zInitialControlVector[0] = startOrientation;

        if (re > 0.0) {
            var spline = segments.get(0).splines.get(0);
            segments.get(0).splines.clear();
            segments.get(0).splines.addAll(Spline3DHelper.subDivide(spline, re, (1 - rs)));

            // Set the orientation component w(i,theta) of a waypoint w(i) to the average value of the
            // adjacent segment orientations, w(i,theta) = 1/2 * (orientation(i-1) + orientation(i)).
            segments.get(0).splines.get(0).zInitialControlVector[0] = startOrientation;
            segments.get(0).splines.get(0).zFinalControlVector[0] = Angles.normalizeAngle(segments.get(0).orientation);
            segments.get(0).splines.get(0).zFinalControlVector[1] = 0.0;

            var last_spline = segments.get(0).splines.size() - 1;
//            segments.get(0).splines.get(last_spline-1).zInitialControlVector[0] = segments.get(0).splines.get(0).zFinalControlVector[0];
//            segments.get(0).splines.get(last_spline-1).zFinalControlVector[0] = segments.get(0).orientation;
//            segments.get(0).splines.get(last_spline-1).zFinalControlVector[1] = 0.0;

//            segments.get(0).splines.get(1).zInitialControlVector[0] = segments.get(0).splines.get(0).zFinalControlVector[0];
//            segments.get(0).splines.get(1).zInitialControlVector[1] = segments.get(0).splines.get(0).zFinalControlVector[1];
//            segments.get(0).splines.get(1).zFinalControlVector[0] = segments.get(0).splines.get(1).zInitialControlVector[0];
//            segments.get(0).splines.get(1).zFinalControlVector[1] = 0.0;

            segments.get(0).splines.get(last_spline).zInitialControlVector[0] = Angles.normalizeAngle(segments.get(0).orientation);

            // At the waypoints, we set the first derivative to point into the direction of rotation,
            // T(i) = e(i) * 1/2 * (theta(i) - theta(i-1), where e(i) is an elongation factor.
            // At initial waypoint use start or segment orientation
            segments.get(0).splines.get(last_spline).zFinalControlVector[1] = 0.0;

            segments.get(0).splines.get(0).zInitialControlVector[1] = 0.5 * optParams.get(0).theta_elongation *
                    Math.min(startOrientation,
                            Angles.shortest_angular_distance(startOrientation, segments.get(0).orientation));
            // Note loop for inner segments will fix up dtheta1 for incoming rotation at inner waypoint
        }

        // Inner segments
        for (int i = 1; i < segments.size(); i++) {
            rs = segments.get(i).rotation_start;
            re = segments.get(i).rotation_end;

            if ((re > 0.0) || (rs > 0.0)) {
                var spline = segments.get(i).splines.get(0);
                segments.get(i).splines.clear();
                segments.get(i).splines.addAll(Spline3DHelper.subDivide(spline, re, (1 - rs)));

                // outgoing control point from segment start
                if (re > 0) {
                    // Set the orientation component w(i,theta) of a waypoint w(i) to the average value of the
                    // adjacent segment orientations, w(i,theta) = 1/2 * (orientation(i-1) + orientation(i)).
                    segments.get(i).splines.get(0).zInitialControlVector[0] =
                            0.5 * Angles.normalizeAngle(segments.get(i - 1).orientation + segments.get(i).orientation);

                    // At the waypoints, we set the first derivative to point into the direction of rotation,
                    // T(i) = e(i) * 1/2 * (theta(i) - theta(i-1), where e(i) is an elongation factor.
                    segments.get(i).splines.get(0).zInitialControlVector[1] = optParams.get(i).theta_elongation *
                            0.5 * Angles.normalizeAngle(segments.get(i).orientation - segments.get(i - 1).orientation);
                }

                if (rs > 0.0 && (i < (segments.size() - 1))) {
                    last = segments.get(i).splines.size() - 1;
                    // Set the orientation component w(i,theta) of a waypoint w(i) to the average value of the
                    // adjacent segment orientations, w(i,theta) = 1/2 * (orientation(i-1) + orientation(i)).
                    segments.get(i).splines.get(last).zFinalControlVector[0] =
                            0.5 * Angles.normalizeAngle(segments.get(i).orientation + segments.get(i + 1).orientation);

                    // At the waypoints, we set the first derivative to point into the direction of rotation,
                    // T(i) = e(i) * 1/2 * (theta(i) - theta(i-1), where e(i) is an elongation factor.
                    segments.get(i).splines.get(last).zFinalControlVector[1] = optParams.get(i).theta_elongation *
                            0.5 * Angles.normalizeAngle(segments.get(i + 1).orientation - segments.get(i).orientation);
                }
            }
        }

        // Fix up last segment's incoming rotation to last waypoint
        last = segments.size() - 1;
        var last_spline = segments.get(last).splines.size() - 1;
        segments.get(last).splines.get(last_spline).zFinalControlVector[0] = Angles.normalizeAngle(endOrientation);
        segments.get(last).splines.get(last_spline).zInitialControlVector[1] = 0.5 * optParams.get(last).theta_elongation *
                Math.min(endOrientation,
                        Angles.shortest_angular_distance(endOrientation, segments.get(last).orientation));

    }

    private static void applyRotation1(
            List<Segment> segments,
            double startOrientation,
            double endOrientation,
            List<OptimizationParameters> optParams) {

        // First segment
        // re distributes on the outgoing segment from the waypoint w(i)
        double re = segments.get(0).rotation_end;

        // rs  distributes on the incoming segment to the next waypoint w(i+1)
        double rs = segments.get(0).rotation_start;

        // set start orientation
        segments.get(0).splines.get(0).zInitialControlVector[0] = startOrientation;
        segments.get(0).splines.get(0).zFinalControlVector[0] = segments.get(0).orientation;

        if ((re > 0.0) || (rs > 0.0)) {
            var spline = segments.get(0).splines.get(0);
            segments.get(0).splines.clear();
            segments.get(0).splines.addAll(Spline3DHelper.subDivide(spline, re, (1 - rs)));

            segments.get(0).splines.get(0).zInitialControlVector[1] = 0.0;

            // Set the orientation component w(i,theta) of a waypoint w(i) to the average value of the
            // adjacent segment orientations, w(i,theta) = 1/2 * (orientation(i-1) + orientation(i)).
//            segments.get(0).splines.get(0).zInitialControlVector[0] = startOrientation;
//            segments.get(0).splines.get(0).zFinalControlVector[0] = Util.normalize_angle(segments.get(0).orientation);
            segments.get(0).splines.get(0).zFinalControlVector[1] = 0.0;

            segments.get(0).splines.get(1).zInitialControlVector[0] = segments.get(0).splines.get(0).zFinalControlVector[0];
            segments.get(0).splines.get(1).zInitialControlVector[1] = segments.get(0).splines.get(0).zFinalControlVector[1];

//            var last_spline = segments.get(0).splines.size() - 1;
//            segments.get(0).splines.get(last_spline-1).zInitialControlVector[0] = segments.get(0).splines.get(0).zFinalControlVector[0];
//            segments.get(0).splines.get(last_spline-1).zFinalControlVector[0] = segments.get(0).orientation;
//            segments.get(0).splines.get(last_spline-1).zFinalControlVector[1] = 0.0;

//            segments.get(0).splines.get(1).zInitialControlVector[0] = segments.get(0).splines.get(0).zFinalControlVector[0];
//            segments.get(0).splines.get(1).zInitialControlVector[1] = segments.get(0).splines.get(0).zFinalControlVector[1];
//            segments.get(0).splines.get(1).zFinalControlVector[0] = segments.get(0).splines.get(1).zInitialControlVector[0];
//            segments.get(0).splines.get(1).zFinalControlVector[1] = 0.0;

//            segments.get(0).splines.get(last_spline).zInitialControlVector[0] = Util.normalize_angle(segments.get(0).orientation);
//
//            // At the waypoints, we set the first derivative to point into the direction of rotation,
//            // T(i) = e(i) * 1/2 * (theta(i) - theta(i-1), where e(i) is an elongation factor.
//            // At initial waypoint use start or segment orientation
//            segments.get(0).splines.get(last_spline).zFinalControlVector[1] = 0.0;
//
//            segments.get(0).splines.get(0).zInitialControlVector[1] = 0.5 * optParams.get(0).theta_elongation *
//                    Math.min(startOrientation,
//                            Util.shortest_angular_distance(startOrientation, segments.get(0).orientation));
            // Note loop for inner segments will fix up dtheta1 for incoming rotation at inner waypoint
        }

        // Inner segments
        for (int i = 1; i < segments.size(); i++) {
            rs = segments.get(i).rotation_start;
            re = segments.get(i).rotation_end;

            if ((re > 0.0) || (rs > 0.0)) {
                var spline = segments.get(i).splines.get(0);
                segments.get(i).splines.clear();
                segments.get(i).splines.addAll(Spline3DHelper.subDivide(spline, re, (1 - rs)));

                // outgoing control point from segment start
                if (re > 0) {
                    // Set the orientation component w(i,theta) of a waypoint w(i) to the average value of the
                    // adjacent segment orientations, w(i,theta) = 1/2 * (orientation(i-1) + orientation(i)).
                    segments.get(i).splines.get(0).zInitialControlVector[0] =
                            0.5 * Angles.normalizeAngle(segments.get(i-1).orientation + segments.get(i).orientation);

                    // At the waypoints, we set the first derivative to point into the direction of rotation,
                    // T(i) = e(i) * 1/2 * (theta(i) - theta(i-1), where e(i) is an elongation factor.
                    segments.get(i).splines.get(0).zInitialControlVector[1] =
                            optParams.get(i).theta_elongation *
                            0.5 * Angles.normalizeAngle(segments.get(i).orientation - segments.get(i-1).orientation);


                    segments.get(i).splines.get(0).zFinalControlVector[0] = segments.get(i).orientation;
                    segments.get(i).splines.get(0).zFinalControlVector[1] = 0.0;
                    segments.get(i).splines.get(0).zFinalControlVector[2] = 0.0;
                    segments.get(i).splines.get(1).zInitialControlVector[0] = segments.get(i).orientation;
                    segments.get(i).splines.get(1).zInitialControlVector[1] = 0.0;
                    segments.get(i).splines.get(1).zInitialControlVector[2] = 0.0;

                    // fix up previous segment angle
                    var prev = i - 1;
                    var prevSpline = segments.get(prev).splines.size() - 1;
                    var value =  segments.get(i).splines.get(0).zInitialControlVector[0];
                    segments.get(prev).splines.get(prevSpline).zFinalControlVector[0] = value;
                    value =  segments.get(i).splines.get(0).zInitialControlVector[1];
                    segments.get(prev).splines.get(prevSpline).zFinalControlVector[1] = value;
                }
            }
        }

        // Fix up last segment's incoming rotation to last waypoint
        var last = segments.size() - 1;
        var last_spline = segments.get(last).splines.size() - 1;
        segments.get(last).splines.get(last_spline).zFinalControlVector[0] = endOrientation;
        segments.get(last).splines.get(last_spline).zInitialControlVector[1] =
                0.5 * optParams.get(last).theta_elongation *
                Math.min(endOrientation,
                        Angles.shortest_angular_distance(endOrientation, segments.get(last).orientation));

    }


    /**
     * Optimization is used to improve the initial trajectory with respect to a
     * user defined cost function, e.g., time of travel.
     *
     * The optimization procedure proposed in the paper, is based on the update rule of RPROP
     * (Riedmiller and Braun, 1993), which is a derivative-free optimization algorithm known
     * for its robust convergence.
     *
     * @param points                    The path as waypoints defined by the initial path planer.
     * @param startOrientation          The start orientation
     * @param endOrientation            The end orientation
     * @param config                    The configuration for generating a trajectory. These are kino-dynamic
     *                                  parameters and constraints.
     * @param initialDeltaCostEpsilon   Initial cost difference.
     * @param minDeltaCostEpsilon       Minimal cost difference.
     * @param initialDParam             Initial D parameter.
     * @param splines                   Used to return best splines.
     * @return An optimized Trajectory.
     */
    public static Trajectory RProp(
            List<TrajectoryPoint> points,
            double startOrientation,
            double endOrientation,
            TrajectoryConfig config,
            double initialDeltaCostEpsilon,
            double minDeltaCostEpsilon,
            double initialDParam,
            List<QuinticHermiteSpline3D> splines
    ) {

        // Create optimization parameters for each segment
        List<OptimizationParameters> optParams = new ArrayList<>();
        List<OptimizationParameters> bestOptParams = new ArrayList<>();
        for (int i = 0; i < points.size(); i++) {
            bestOptParams.add(new OptimizationParameters());
        }

        List<QuinticHermiteSpline3D> bestSpline = generateSpline(points, startOrientation, endOrientation, bestOptParams);
        Trajectory bestTrajectory = evaluateTrajectory(bestSpline, config);
        double bestCost = bestTrajectory.getTotalTimeSeconds();
        double deltaCostEpsilon = initialDeltaCostEpsilon;

        while (deltaCostEpsilon >= minDeltaCostEpsilon) {
            boolean bestCostChanged = false;

            // Start with the previous best optimization parameters, then loop through each and try
            // to improve them.  Ignore the first and last point - can't  optimize the starting and
            // end position since those need match exactly.
            int last = bestOptParams.size() - 1;
            for (int i = 0; i < bestOptParams.size(); i++) {
                for (int j = 0; j < bestOptParams.get(i).size(); j++) {
                    // first waypoint doesn't have rs
                    if ((i == 0) &&  (j == 1)) {
                        continue;
                    }
                    // last waypoint doesn't have re
                    if ((i == last) &&  (j == 2)) {
                        continue;
                    }

                    // Copy optimization parameters
                    optParams.clear();
                    for (OptimizationParameters p : bestOptParams) {
                        optParams.add(new OptimizationParameters(p));
                    }

                    double deltaCost = Double.POSITIVE_INFINITY;
                    double currCost = bestCost;
                    double dparam = initialDParam;

                    if (j == 0) {
                        dparam = 0.25;
                    }
                    if ((j == 1) || (j == 2)) {
                        dparam = 0.15;
                    }

                    // One exit criteria for the inner loop is if the cost stops improving by an appreciable
                    // amount while changing this one parameter. Track that here
                    while (deltaCost > deltaCostEpsilon) {
                        double value = optParams.get(i).get(j) + dparam;
                        // elongation cannot be less than 0
                        if ((j == 0) || (j == 3)) {
                            value = Math.max(0.0, value);
                            optParams.get(i).set(j, value);
                        } else if (j == 4) {
                            //theta_lambda
                            value = Math.max(0.0, Math.min(1.0, value));
//                            value = Math.max(0.0, value);
                            for (OptimizationParameters o : optParams) {
                                o.set(j, value);
                            }
                        } else if ((j == 1) || (j == 2)) {
                            // clamp rs and re values
                            if (j == 1) {
                                value = Math.max(0.0, Math.min(value, 1 - optParams.get(i).get(2)));
                            }
                            if (j == 2) {
                                value = Math.max(0.0, Math.min(value, 1 - optParams.get(i).get(1)));
                            }
                            optParams.get(i).set(j, value);
                        }

//                        System.out.println("optParams");
//                        for (OptimizationParameters o : optParams) {
//                            System.out.println(o.toString());
//                        }
//                        System.out.println();

                        List<QuinticHermiteSpline3D> thisSpline = generateSpline(points, startOrientation, endOrientation, optParams);
                        Trajectory thisTrajectory = evaluateTrajectory(thisSpline, config);
                        double thisCost = thisTrajectory.getTotalTimeSeconds();

                        // If cost is better than the best cost, record it and
                        // move on to optimizing the next parameter. It is possible
                        // to loop back and return to this one again, but the paper
                        // says that hyper-optimizing one parameter before moving
                        // to others can lead to getting stuck in local minima.
                        if (thisCost < bestCost) {
                            System.out.printf("best params [%d][ %d], bestCost=%f, thisCost=%f, " +
                                    "%f, %f%n", i, j, bestCost, thisCost, thisCost-bestCost, dparam);
                            bestOptParams.clear();
                            for (var optParam: optParams) {
                                bestOptParams.add(optParam);
                            }
                            bestTrajectory = thisTrajectory;
                            bestCost = thisCost;
                            bestCostChanged = true;
                            bestSpline.clear();
                            bestSpline.addAll(thisSpline);
//                            System.out.println("BestParams");
//                            for (OptimizationParameters o : bestOptParams) {
//                                System.out.println(o.toString());
//                            }

                            break;
                        }

                        // Use sign of difference between this cost and the previous one
                        // to adjust the dparam value added to the parameter being optimized.
                        // 1.2 and -0.5 are intentionally not factors of each other to prevent
                        //  oscillating between the same set of values.
                        if (thisCost < currCost) {
                            dparam *= 1.2;
                        } else {
                            dparam *= -0.5;
                        }

                        // Record values for next iteration
                        deltaCost = Math.abs(thisCost - currCost);
//                        System.out.println(
//                                String.format("RProp: i=%d, j=%d, ", i, j) +
//                                String.format("bestCost=%f, thisCost=%f, ", bestCost, thisCost) +
//                                String.format("currCost=%f, deltaCost=%f, deltaCostEpsilon=%f", currCost, deltaCost, deltaCostEpsilon)
//                        );

                        currCost = thisCost;

                    }
                }
            }
            if (!bestCostChanged) {
                deltaCostEpsilon /= 1.75;
            }
        }

        // Return values
        splines.clear();
        splines.addAll(bestSpline);
        return bestTrajectory;
    }

    public static Trajectory evaluateTrajectory(
            List<QuinticHermiteSpline3D> splines,
            TrajectoryConfig config) {

        Trajectory trajectory = TrajectoryGenerator.generateTrajectoryFromSplines(splines, config);

        return trajectory;
    }

    public static void set_first_last_point_acceleration(
            List<Double> p1, List<Double> p2,
            List<Double> v1, List<Double> v2,
            List<Double> accelerations) {

        if (accelerations.size() >= 3) {
            return;
        }

        double Ax = p1.get(0);
        double Ay = p1.get(1);
        double tAx = v1.get(0);
        double tAy = v1.get(1);
        double Bx = p2.get(0);
        double By = p2.get(1);
        double tBx = v2.get(0);
        double tBy = v2.get(1);

        double xaccel = 6.0 * Ax + 2.0 * tAx + 4.0 * tBx - 6.0 * Bx;
        double yaccel = 6.0 * Ay + 2.0 * tAy + 4.0 * tBy - 6.0 * By;

        if (accelerations.size() == 0) {
            accelerations.add(xaccel);
        }

        if (accelerations.size() == 1) {
            accelerations.add(yaccel);
        }

        if (accelerations.size() == 2) {
            accelerations.add(0.0);
        }
    }
     
    public static double get_first_last_rotation_control_acceleration(
            double theta0, double theta1, double dtheta0, double dtheta1) {
        return 6.0 * theta0 + 2.0 * dtheta0 + 4.0 * dtheta1 - 6.0 * theta1;
    }
}