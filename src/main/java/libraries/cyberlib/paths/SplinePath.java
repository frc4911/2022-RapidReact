package libraries.cyberlib.paths;

import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cyberlib.spline.QuinticHermiteSpline3D;
import libraries.cyberlib.trajectory.Trajectory;
import libraries.cyberlib.trajectory.TrajectoryConfig;

import java.util.ArrayList;
import java.util.List;

import static libraries.cyberlib.trajectory.swerve.SwerveTrajectoryGenerator.create_trajectory;

public class SplinePath {

    public final List<Translation2d> waypoints;
    public final Rotation2d startHeading;
    public final Rotation2d endHeading;

    public SplinePath(List<Translation2d> waypoints, Rotation2d startHeading, Rotation2d endHeading) {
        this.waypoints = waypoints;
        this.startHeading = startHeading;
        this.endHeading = endHeading;
    }

    public Trajectory generateTrajectory(TrajectoryConfig config) {
        List<QuinticHermiteSpline3D> splines = new ArrayList<>();

        var trajectory = create_trajectory(
                waypoints,
                startHeading.getRadians(),
                endHeading.getRadians(),
                config,
                splines
        );

        return trajectory;
    }
}
