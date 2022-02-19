package libraries.cyberlib.spline;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class CubicHermiteSpline3DTest {
    private static final double kMaxDx = 0.127;
    private static final double kMaxDy = 0.00127;
    private static final double kMaxDtheta = 0.0872;

    @SuppressWarnings({"ParameterName", "PMD.UnusedLocalVariable"})
    private void run(Pose2d a, List<Translation2d> waypoints, Pose2d b) {
        // Start the timer.
        //var start = System.nanoTime();

        // Generate and parameterize the spline.
        var controlVectors =
                Spline3DHelper.getCubicControlVectorsFromWaypoints(a,
                        waypoints.toArray(new Translation2d[0]), b);
        var splines
                = Spline3DHelper.getCubicSplinesFromControlVectors(
                controlVectors[0], waypoints.toArray(new Translation2d[0]), controlVectors[1]);

        var poses = new ArrayList<PoseWithCurvatureAndOrientation>();

        poses.add(splines[0].getPoint(0.0));

        for (var spline : splines) {
            poses.addAll(Spline3DParameterizer.parameterize(spline));
        }

        // End the timer.
        //var end = System.nanoTime();

        // Calculate the duration (used when benchmarking)
        //var durationMicroseconds = (end - start) / 1000.0;

        for (int i = 0; i < poses.size() - 1; i++) {
            var p0 = poses.get(i);
            var p1 = poses.get(i + 1);

            // Make sure the twist is under the tolerance defined by the Spline class.
            var twist = p0.poseMeters.log(p1.poseMeters);
            assertAll(
                    () -> assertTrue(Math.abs(twist.dx) < kMaxDx),
                    () -> assertTrue(Math.abs(twist.dy) < kMaxDy),
                    () -> assertTrue(Math.abs(twist.dtheta) < kMaxDtheta)
            );
        }

        // Check first point
        assertAll(
                () -> assertEquals(a.getTranslation().getX(),
                        poses.get(0).poseMeters.getTranslation().getX(), 1E-9),
                () -> assertEquals(a.getTranslation().getY(),
                        poses.get(0).poseMeters.getTranslation().getY(), 1E-9),
                () -> assertEquals(a.getRotation().getRadians(),
                        poses.get(0).poseMeters.getRotation().getRadians(), 1E-9)
        );

        // Check interior waypoints
        boolean interiorsGood = true;
        for (var waypoint : waypoints) {
            boolean found = false;
            for (var state : poses) {
                if (waypoint.getDistance(state.poseMeters.getTranslation()) == 0) {
                    found = true;
                }
            }
            interiorsGood &= found;
        }

        assertTrue(interiorsGood);

        // Check last point
        assertAll(
                () -> assertEquals(b.getTranslation().getX(),
                        poses.get(poses.size() - 1).poseMeters.getTranslation().getX(), 1E-9),
                () -> assertEquals(b.getTranslation().getY(),
                        poses.get(poses.size() - 1).poseMeters.getTranslation().getY(), 1E-9),
                () -> assertEquals(b.getRotation().getRadians(),
                        poses.get(poses.size() - 1).poseMeters.getRotation().getRadians(), 1E-9)
        );
    }

    @SuppressWarnings("PMD.JUnitTestsShouldIncludeAssert")
    @Test
    void testStraightLine() {
        run(new Pose2d(), new ArrayList<>(), new Pose2d(3, 0, new Rotation2d()));
    }

    @SuppressWarnings("PMD.JUnitTestsShouldIncludeAssert")
    @Test
    void testSCurve() {
        var start = new Pose2d(0, 0, Rotation2d.fromDegrees(90.0));
        ArrayList<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(new Translation2d(1, 1));
        waypoints.add(new Translation2d(2, -1));
        var end = new Pose2d(3, 0, Rotation2d.fromDegrees(90.0));

        run(start, waypoints, end);
    }

    @SuppressWarnings("PMD.JUnitTestsShouldIncludeAssert")
    @Test
    void testOneInterior() {
        var start = new Pose2d(0, 0, Rotation2d.fromDegrees(0.0));
        ArrayList<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(new Translation2d(2.0, 0.0));
        var end = new Pose2d(4, 0, Rotation2d.fromDegrees(0.0));

        run(start, waypoints, end);
    }

    @SuppressWarnings("PMD.JUnitTestsShouldIncludeAssert")
    @Test
    void testWindyPath() {
        final var start = new Pose2d(0, 0, Rotation2d.fromDegrees(0.0));
        final ArrayList<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(new Translation2d(0.5, 0.5));
        waypoints.add(new Translation2d(0.5, 0.5));
        waypoints.add(new Translation2d(1.0, 0.0));
        waypoints.add(new Translation2d(1.5, 0.5));
        waypoints.add(new Translation2d(2.0, 0.0));
        waypoints.add(new Translation2d(2.5, 0.5));
        final var end = new Pose2d(3.0, 0.0, Rotation2d.fromDegrees(0.0));

        run(start, waypoints, end);
    }

    @Test
    void testMalformed() {
        assertThrows(Spline3DParameterizer.MalformedSplineException.class, () -> run(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new ArrayList<>(), new Pose2d(1, 0, Rotation2d.fromDegrees(180))));
        assertThrows(Spline3DParameterizer.MalformedSplineException.class, () -> run(
                new Pose2d(10, 10, Rotation2d.fromDegrees(90)),
                Arrays.asList(new Translation2d(10, 10.5)),
                new Pose2d(10, 11, Rotation2d.fromDegrees(-90))));
    }
}
