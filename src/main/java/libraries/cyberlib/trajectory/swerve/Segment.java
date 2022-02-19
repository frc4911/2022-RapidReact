package libraries.cyberlib.trajectory.swerve;

import libraries.cyberlib.spline.QuinticHermiteSpline3D;
import libraries.cyberlib.spline.Spline3D;
import libraries.cyberlib.trajectory.TrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Represents a path between two waypoints that we want to optimize.
 */
public class Segment {
    public List<QuinticHermiteSpline3D> splines;
    public double segment_length;
    public double angle;
    public double path_fraction;
    public double initial_orientation;
    public double minimized_orientation;
    public double orientation;
    public Optional<QuinticHermiteSpline3D> spline;
    public double rotation_start;
    public double rotation_end;
    public Spline3D.ControlVector startPoint;
    public Spline3D.ControlVector endPoint;
    public TrajectoryGenerator.ControlVectorList controlVectors;

    public Segment() {
        this.splines =  new ArrayList<>();
        this.segment_length = 0.0;
        this.angle = -999.0;
        this.path_fraction = 0.0;
        this.initial_orientation = 0.0;
        this.minimized_orientation = -999.0;
        this.orientation = 0.0;
        this.spline = Optional.empty();
        this.rotation_start = 0.0;
        this.rotation_end = 0.0;
        this.controlVectors = new TrajectoryGenerator.ControlVectorList();
    }

    @Override
    public String toString() {
        return String.format("length=%.3f, ", this.segment_length) +
                String.format("angle=%.3f (%.3f), ", this.angle, Math.toDegrees(this.angle)) +
                String.format("frac=%.3f, ", this.path_fraction) +
                String.format("init=%.3f (%.3f), ", this.initial_orientation, Math.toDegrees((this.initial_orientation))) +
                String.format("ori=%.3f (%.3f), ", this.orientation, Math.toDegrees(this.orientation)) +
                String.format("min=%.3f (%.3f), ", this.minimized_orientation, Math.toDegrees(this.minimized_orientation)) +
                String.format("rot start=%.3f, rot end=%.3f, ", this.rotation_start, this.rotation_end) +
                String.format("spines=%s ", splines.size());
    }
}
