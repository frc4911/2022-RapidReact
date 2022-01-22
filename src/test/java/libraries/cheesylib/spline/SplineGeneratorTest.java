package libraries.cheesylib.spline;

import libraries.cheesylib.geometry.*;
import libraries.cheesylib.util.Util;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;

public class SplineGeneratorTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void test() {
        // Create the test spline
        Pose2d p1 = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        Pose2d p2 = new Pose2d(new Translation2d(15, 10), new Rotation2d(1, -5, true));
        Spline s = new QuinticHermiteSpline(p1, p2);

        List<Pose2dWithCurvature> samples = SplineGenerator.parameterizeSpline(s);

        double arclength = 0;
        Pose2dWithCurvature cur_pose = samples.get(0);
        for (Pose2dWithCurvature sample : samples) {
            final Twist2d t = Pose2d.log(cur_pose.getPose().inverse().transformBy(sample.getPose()));
            arclength += t.dx;
            cur_pose = sample;
        }

        assertEquals(cur_pose.getTranslation().x(), 15.0, kTestEpsilon);
        assertEquals(cur_pose.getTranslation().y(), 10.0, kTestEpsilon);
        assertEquals(cur_pose.getRotation().getDegrees(), -78.69006752597981, kTestEpsilon);
        // assertEquals(arclength, 23.17291953186379, kTestEpsilon);
    }

//    @Test
    public void test2() {
        // Create the test spline
        List<Pose2d> wayPoints = new ArrayList<>();
        wayPoints.add(new Pose2d(new Translation2d(0, 0), new Rotation2d()));
        wayPoints.add(new Pose2d(new Translation2d(20, 20), Rotation2d.fromDegrees(90)));
        wayPoints.add(new Pose2d(new Translation2d(40, 0), Rotation2d.fromDegrees(0)));

        List<QuinticHermiteSpline> splines = new ArrayList<>(wayPoints.size() - 1);
        for (int i = 1; i < wayPoints.size(); i++) {
            splines.add(new QuinticHermiteSpline(wayPoints.get(i - 1), wayPoints.get(i)));
        }

        QuinticHermiteSpline.optimizeSpline(splines);
        List<Spline> splines2 = new ArrayList<>(splines);
        List<Pose2dWithCurvature> positions = SplineGenerator.parameterizeSplines(splines2);

        for (Pose2dWithCurvature p : positions) {
            System.out.format("pos, x=%.7f, y=%7f",
                    p.getPose().getTranslation().x(),
                    p.getPose().getTranslation().y());
            System.out.println();
        }
    }
}