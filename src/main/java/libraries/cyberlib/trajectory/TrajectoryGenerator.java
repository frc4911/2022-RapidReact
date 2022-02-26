package libraries.cyberlib.trajectory;

import libraries.cyberlib.spline.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public final class TrajectoryGenerator {
    private static final Trajectory kDoNothingTrajectory =
            new Trajectory(List.of(new Trajectory.State()));

    /**
     * Private constructor because this is a utility class.
     */
    private TrajectoryGenerator() {
    }
    
    /**
     * Set error reporting function. By default, DriverStation.reportError() is used.
     *
     * @param func Error reporting function, arguments are error and stackTrace.
     */
//    public static void setErrorHandler(BiConsumer<String, StackTraceElement[]> func) {
//        errorFunc = func;
//    }

    /**
     * Generates a trajectory from the given control vectors and config. This method uses clamped
     * cubic splines -- a method in which the exterior control vectors and interior waypoints
     * are provided. The headings are automatically determined at the interior points to
     * ensure continuous curvature.
     *
     * @param initial           The initial control vector.
     * @param interiorWaypoints The interior waypoints.
     * @param end               The ending control vector.
     * @param config            The configuration for the trajectory.
     * @return The generated trajectory.
     */
    public static Trajectory generateTrajectory(
            Spline3D.ControlVector initial,
            List<Translation2d> interiorWaypoints,
            Spline3D.ControlVector end,
            TrajectoryConfig config) {
        final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));

        // Clone the control vectors.
        var newInitial = new Spline3D.ControlVector(initial.x, initial.y, initial.z);
        var newEnd = new Spline3D.ControlVector(end.x, end.y, end.z);

        // Change the orientation if reversed.
        if (config.isReversed()) {
            newInitial.x[1] *= -1;
            newInitial.y[1] *= -1;
            newInitial.z[1] *= -1;
            newEnd.x[1] *= -1;
            newEnd.y[1] *= -1;
            newEnd.z[1] *= -1;
        }

        // Get the spline points
        List<PoseWithCurvatureAndOrientation> points;
        try {
            points = 
				splinePointsFromSplines(Spline3DHelper.getCubicSplinesFromControlVectors(
                    newInitial, interiorWaypoints.toArray(new Translation2d[0]), newEnd));
        } catch (Spline3DParameterizer.MalformedSplineException ex) {
//            reportError(ex.getMessage(), ex.getStackTrace());
            return kDoNothingTrajectory;
        }

        // Change the points back to their original orientation.
        if (config.isReversed()) {
            for (var point : points) {
                point.poseMeters = point.poseMeters.plus(flip);
                point.curvatureRadPerMeter *= -1;
            }
        }

        // Generate and return trajectory.
        return TrajectoryParameterizer.timeParameterizeTrajectory(
			points, 
			config.getConstraints(),
            config.getStartVelocity(), 
			config.getEndVelocity(), 
			config.getMaxVelocity(),
			config.getMaxAcceleration(),
            config.getStartAngularVelocity(), config.getEndAngularVelocity(),
            config.getMaxAngularVelocity(),
            config.getMaxAngularAcceleration(),
            config.isReversed());
    }

    /**
     * Generates a trajectory from the given waypoints and config. This method uses clamped
     * cubic splines -- a method in which the initial pose, final pose, and interior waypoints
     * are provided.  The headings are automatically determined at the interior points to
     * ensure continuous curvature.
     *
     * @param start             The starting pose.
     * @param interiorWaypoints The interior waypoints.
     * @param end               The ending pose.
     * @param config            The configuration for the trajectory.
     * @return The generated trajectory.
     */
    public static Trajectory generateTrajectory(
            Pose2d start, 
            List<Translation2d> interiorWaypoints, 
            Pose2d end, 
            TrajectoryConfig config) {
        var controlVectors = Spline3DHelper.getCubicControlVectorsFromWaypoints(
                start, interiorWaypoints.toArray(new Translation2d[0]), end
        );

        // Return the generated trajectory.
        return generateTrajectory(controlVectors[0], interiorWaypoints, controlVectors[1], config);
    }

    /**
     * Generates a trajectory from the given quintic control vectors and config. This method
     * uses quintic hermite splines -- therefore, all points must be represented by control
     * vectors. Continuous curvature is guaranteed in this method.
     *
     * @param controlVectors List of quintic control vectors.
     * @param config         The configuration for the trajectory.
     * @return The generated trajectory.
     */
    public static Trajectory generateTrajectory(ControlVectorList controlVectors, TrajectoryConfig config) {
        final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));
        final var newControlVectors = new ArrayList<Spline3D.ControlVector>(controlVectors.size());

        // Create a new control vector list, flipping the orientation if reversed.
        for (final var vector : controlVectors) {
            var newVector = new Spline3D.ControlVector(vector.x, vector.y, vector.z);
            if (config.isReversed()) {
                newVector.x[1] *= -1;
                newVector.y[1] *= -1;
                newVector.z[1] *= -1;
            }
            newControlVectors.add(newVector);
        }

        // Get the spline points
        List<PoseWithCurvatureAndOrientation> points;
        try {
            points = 
				splinePointsFromSplines(
					Spline3DHelper.getQuinticSplinesFromControlVectors(
                    	newControlVectors.toArray(new Spline3D.ControlVector[]{})));
        } catch (Spline3DParameterizer.MalformedSplineException ex) {
//            reportError(ex.getMessage(), ex.getStackTrace());
            return kDoNothingTrajectory;
        }

        // Change the points back to their original orientation.
        if (config.isReversed()) {
            for (var point : points) {
                point.poseMeters = point.poseMeters.plus(flip);
                point.curvatureRadPerMeter *= -1;
            }
        }

        // Generate and return trajectory.
        return TrajectoryParameterizer.timeParameterizeTrajectory(points, config.getConstraints(),
                config.getStartVelocity(), config.getEndVelocity(), config.getMaxVelocity(),
                config.getMaxAcceleration(),
                config.getStartAngularVelocity(), config.getEndAngularVelocity(),
                config.getMaxAngularVelocity(),
                config.getMaxAngularAcceleration(),
                config.isReversed());
    }

    /**
     * Generates a trajectory from the given waypoints and config. This method
     * uses quintic hermite splines -- therefore, all points must be represented by Pose2d
     * objects. Continuous curvature is guaranteed in this method.
     *
     * @param waypoints List of waypoints.
     * @param config    The configuration for the trajectory.
     * @return The generated trajectory.
     */
    @SuppressWarnings("LocalVariableName")
    public static Trajectory generateTrajectory(List<Pose2d> waypoints, TrajectoryConfig config) {
        final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));

        List<Pose2d> newWaypoints = new ArrayList<>();
        if (config.isReversed()) {
            for (Pose2d originalWaypoint : waypoints) {
                newWaypoints.add(originalWaypoint.plus(flip));
            }
        } else {
            newWaypoints.addAll(waypoints);
        }

        // Get the spline points
        List<PoseWithCurvatureAndOrientation> points;
        try {
            points = splinePointsFromSplines(Spline3DHelper.getQuinticSplinesFromWaypoints(newWaypoints));
        } catch (Spline3DParameterizer.MalformedSplineException ex) {
//            reportError(ex.getMessage(), ex.getStackTrace());
            return kDoNothingTrajectory;
        }

        // Change the points back to their original orientation.
        if (config.isReversed()) {
            for (var point : points) {
                point.poseMeters = point.poseMeters.plus(flip);
                point.curvatureRadPerMeter *= -1;
            }
        }

        // Generate and return trajectory.
        return TrajectoryParameterizer.timeParameterizeTrajectory(
                points,
                config.getConstraints(),
                config.getStartVelocity(),
                config.getEndVelocity(),
                config.getMaxVelocity(),
                config.getMaxAcceleration(),
                config.getStartAngularVelocity(),
                config.getEndAngularVelocity(),
                config.getMaxAngularVelocity(),
                config.getMaxAngularAcceleration(),
                config.isReversed());
    }

    /**
     * Generates a trajectory from the given spines and config. This method
     * uses quintic hermite splines.  Continuous curvature is guaranteed in this method.
     *
     * @param splines   List of splines.
     * @param config    The configuration for the trajectory.
     * @return The generated trajectory.
     */
    @SuppressWarnings("LocalVariableName")
    public static Trajectory generateTrajectoryFromSplines(
            List<QuinticHermiteSpline3D> splines, 
            TrajectoryConfig config) {
        final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));

        // TODO:  Add flip support

        // Get the spline points
        List<PoseWithCurvatureAndOrientation> points;
        try {
            points = splinePointsFromSplines(splines.toArray(Spline3D[]::new));
        } catch (Spline3DParameterizer.MalformedSplineException ex) {
//            reportError(ex.getMessage(), ex.getStackTrace());
            return kDoNothingTrajectory;
        }

        // Generate and return trajectory.
        return TrajectoryParameterizer.timeParameterizeTrajectory(
                points,
                config.getConstraints(),
                config.getStartVelocity(),
                config.getEndVelocity(),
                config.getMaxVelocity(),
                config.getMaxAcceleration(),
                config.getStartAngularVelocity(),
                config.getEndAngularVelocity(),
                config.getMaxAngularVelocity(),
                config.getMaxAngularAcceleration(),
                config.isReversed());
    }

    /**
     * Generate spline points from a vector of splines by parameterizing the
     * splines.
     *>
     * @param splines The splines to parameterize.
     * @return The spline points for use in time parameterization of a trajectory.
     * @throws Spline3DParameterizer.MalformedSplineException When the spline is malformed (e.g. has close adjacent
     *                                                        points with approximately opposing headings)
     */
    public static List<PoseWithCurvatureAndOrientation> splinePointsFromSplines(Spline3D[] splines) {
        // Create the vector of spline points.
        var splinePoints = new ArrayList<PoseWithCurvatureAndOrientation>();

        // Add the first point to the vector.
        splinePoints.add(splines[0].getPoint(0.0));

        // Iterate through the vector and parameterize each spline, adding the
        // parameterized points to the final vector.
        for (final var spline : splines) {
//            System.out.println(spline.toString());
            var points = Spline3DParameterizer.parameterize(spline);

            // Append the array of poses to the vector. We are removing the first
            // point because it's a duplicate of the last point from the previous
            // spline.
            splinePoints.addAll(points.subList(1, points.size()));
        }
        return splinePoints;
    }

    // Work around type erasure signatures
    public static class ControlVectorList extends ArrayList<Spline3D.ControlVector> {
        public ControlVectorList(int initialCapacity) {
            super(initialCapacity);
        }

        public ControlVectorList() {
            super();
        }

        public ControlVectorList(Collection<? extends Spline3D.ControlVector> collection) {
            super(collection);
        }
    }
}