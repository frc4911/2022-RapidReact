package libraries.cyberlib.paths;

import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;
import libraries.cyberlib.spline.Spline3D;
import libraries.cyberlib.spline.Spline3DParameterizer;

import java.util.List;

public class Spline3DSegment extends PathSegment {
    private static final double LENGTH_SAMPLE_STEP = 1.0e-4;

    private final Spline3D spline;

    private transient double length = Double.NaN;

    public Spline3DSegment(Spline3D spline) {
        this.spline = spline;
    }

    @Override
    public PoseWithCurvatureAndOrientation getPoint(double t) {
        return spline.getPoint(t);
    }

    @Override
    public double getLength() {
        if (!Double.isFinite(length)) {
            length = 0.0;
            var  p0 = spline.getPoint(0.0).poseMeters.getTranslation();
            for (double t = LENGTH_SAMPLE_STEP; t <= 1.0; t += LENGTH_SAMPLE_STEP) {
                var p1 = spline.getPoint(t).poseMeters.getTranslation();
                length += p1.minus(p0).getNorm();

                p0 = p1;
            }
        }

        return length;
    }

    @Override
    public List<PoseWithCurvatureAndOrientation> parameterize() {
        // Use custom parameterizer.
        return Spline3DParameterizer.parameterize(this.spline);
    }
}
