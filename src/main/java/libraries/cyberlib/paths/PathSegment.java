package libraries.cyberlib.paths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import libraries.cyberlib.spline.PoseWithCurvatureAndOrientation;

import java.util.List;

/**
 * Represents a path segment.  Path segments can be chained together to form paths.
 */
public abstract class PathSegment {

    public PoseWithCurvatureAndOrientation getStart() {
        return getPoint(0.0);
    }

    public PoseWithCurvatureAndOrientation getEnd() {
        return getPoint(1.0);
    }

    public abstract PoseWithCurvatureAndOrientation getPoint(double t);

    public abstract List<PoseWithCurvatureAndOrientation> parameterize();

    public abstract double getLength();

//    public static class State {
//
//        private final Translation2d position;
//        private final Rotation2d heading;
//        private final double curvature;
//        private final Rotation2d orientation;
//
//        public State(
//                Translation2d positionInMeters,
//                Rotation2d headingInRadians,
//                double curvature,
//                Rotation2d orientationInRadians) {
//            this.position = positionInMeters;
//            this.heading = headingInRadians;
//            this.curvature = curvature;
//            this.orientation = orientationInRadians;
//        }
//
//        /**
//         * Gets the 2d position of the path at a given point.
//         *
//         * @return the (x,y) point of the at a given point
//         */
//        public Translation2d getPosition() {
//            return position;
//        }
//
//        /**
//         * Gets the heading of the path at a given point.
//         *
//         * @return The heading of the path at a given point.
//         */
//        public Rotation2d getHeading() {
//            return heading;
//        }
//
//        /**
//         * Gets the curvature of the path at a given point.
//         *
//         * @return The heading of the path at a given point
//         */
//        public double getCurvature() {
//            return curvature;
//        }
//
//        /**
//         * Gets the orientation of the robot on the path at a given point.
//         *
//         * @return The orientation of the robot on the path at a given point.
//         */
//        public Rotation2d getOrientation() {
//            return orientation;
//        }
//    }
}
