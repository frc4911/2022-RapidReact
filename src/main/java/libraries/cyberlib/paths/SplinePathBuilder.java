package libraries.cyberlib.paths;


import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a way to build simple and complex paths composed of one or more splines.
 * A spline is defined by at least two positions often called waypoints. The resulting
 * path run through the set of waypoints.
 */
public class SplinePathBuilder {
    private final List<Translation2d> waypoints = new ArrayList<>();
    private final Rotation2d initialHeading;
    private final Rotation2d finalHeading;

    /**
     * Creates a SplinePathBuilder instance with an initial position, initial heading, and final heading.
     *
     * @param initialPosition The initial position
     * @param initialHeading  The initial heading
     * @param finalHeading    The final heading
     */
    public SplinePathBuilder(Translation2d initialPosition, Rotation2d initialHeading, Rotation2d finalHeading) {
        waypoints.add(initialPosition);
        this.initialHeading = initialHeading;
        this.finalHeading = finalHeading;
    }

    public SplinePath build() {
        return new SplinePath(waypoints, initialHeading, finalHeading);
    }

    /**
     * Adds a waypoint to the path.
     *
     * @param waypoint The next position for path.
     * @return An instance of the PathBuilder object.
     */
    public SplinePathBuilder splineTo(Translation2d waypoint) {
        waypoints.add(waypoint);
        return this;
    }
}
