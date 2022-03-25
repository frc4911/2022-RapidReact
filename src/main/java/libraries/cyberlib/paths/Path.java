package libraries.cyberlib.paths;

import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;

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

}
