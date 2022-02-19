package libraries.cyberlib.trajectory.swerve;

import libraries.cheesylib.spline.QuinticHermiteSpline;
import edu.wpi.first.math.util.Units;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cyberlib.kinematics.SwerveDriveKinematics;
import libraries.cyberlib.spline.QuinticHermiteSpline3D;
import libraries.cyberlib.trajectory.TrajectoryConfig;
import libraries.cyberlib.trajectory.TrajectoryUtil;
import libraries.cyberlib.trajectory.constraints.CentripetalAccelerationConstraint;
import libraries.cyberlib.trajectory.constraints.SwerveDriveConstraint;
import libraries.cyberlib.trajectory.constraints.TrajectoryConstraint;
import org.junit.jupiter.api.Test;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static libraries.cyberlib.trajectory.swerve.SwerveTrajectoryGenerator.createTrajectoryPoints;
import static libraries.cyberlib.trajectory.swerve.SwerveTrajectoryGenerator.generateSpline;
import static org.junit.jupiter.api.Assertions.assertEquals;


public class SwerveTrajectoryGeneratorTest {
    final static double kRobotWidth = Units.feetToMeters(30.0 / 12.0);
    final static double kRobotLength = Units.feetToMeters(30.0 / 12.0);
    final static double kRadius = Math.hypot(kRobotWidth/2, kRobotLength/2);

    final static double kVelocity = Units.feetToMeters(14.2);  // feet/s
    final static double kAcceleration = kVelocity / 0.4;  // feet/s*s
    final static double kAngularVelocity = Math.min(kVelocity / kRadius, Math.toRadians((270)));  // radians/s

    final static double kAngularAcceleration = kAngularVelocity / 0.4;

    final static Translation2d front_left  = new Translation2d(kRadius, kRadius);
    final static Translation2d front_right = new Translation2d(kRadius, -kRadius);
    final static Translation2d back_left = new Translation2d(-kRadius, kRadius);
    final static Translation2d back_right = new Translation2d(-kRadius, -kRadius);
    final static SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(front_left, front_right, back_left, back_right);

    @Test
    void checkSplines(){
        List<Translation2d> wayPoints = new ArrayList<>();
        wayPoints.add(new Translation2d(0, 0));
        wayPoints.add(new Translation2d(Units.inchesToMeters(50), Units.inchesToMeters(50)));
        wayPoints.add(new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(10)));
        wayPoints.add(new Translation2d(Units.inchesToMeters(120), 0));
        wayPoints.add(new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(-10)));

        var start_orientation = Math.toRadians(-180.0);
        var end_orientation = Math.toRadians(0.0);

        var q0 = new QuinticHermiteSpline3D(
                new double[] {wayPoints.get(0).getTranslation().x(), 0.5, 1.2},
                new double[] {wayPoints.get(1).getTranslation().x(), 0.5, 1.2},
                new double[] {wayPoints.get(0).getTranslation().y(), 0.5, 1.2},
                new double[] {wayPoints.get(1).getTranslation().y(), 0.5, 1.2},
                new double[] {start_orientation, 0.5, 1.2},
                new double[] {end_orientation, 0.5, 1.2}
        );

        var q1 = new QuinticHermiteSpline(
                wayPoints.get(0).getTranslation().x(), wayPoints.get(1).getTranslation().x(),
                0.5, 0.5,
                1.2, 1.2,
                wayPoints.get(0).getTranslation().y(), wayPoints.get(1).getTranslation().y(),
                0.5, 0.5,
                1.2, 1.2,
                start_orientation, end_orientation,
                0.5, 0.5,
                1.2,1.2
        );

        var pNew = q0.getPoint(0.0).poseMeters;
        var pOld = q1.getPoint(0.0);

        assertEquals(pNew.getTranslation().getX(), pOld.x(),  1E-9);
        assertEquals(pNew.getTranslation().getY(), pOld.y(),  1E-9);

        pNew = q0.getPoint(0.5).poseMeters;
        pOld = q1.getPoint(0.5);

        assertEquals(pNew.getTranslation().getX(), pOld.x(),  1E-9);
        assertEquals(pNew.getTranslation().getY(), pOld.y(),  1E-9);

        pNew = q0.getPoint(1.0).poseMeters;
        pOld = q1.getPoint(1.0);

        System.out.println(
                String.format("pNew.x=%.3f, pOld.x=%.3f, ", pNew.getTranslation().getX(), pOld.x()) +
                String.format("pNew.y=%.3f, pOld.y=%.3f, ", pNew.getTranslation().getY(), pOld.y())
        );

        assertEquals(pNew.getTranslation().getX(), pOld.x(),  1E-9);
        assertEquals(pNew.getTranslation().getY(), pOld.y(),  1E-9);
    }

    @Test
    void generateTrajectory() throws IOException {
        List<Translation2d> wayPoints = new ArrayList<>();
        wayPoints.add(new Translation2d(0, 0));
        wayPoints.add(new Translation2d(Units.inchesToMeters(50), Units.inchesToMeters(50)));
        wayPoints.add(new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(10)));
        wayPoints.add(new Translation2d(Units.inchesToMeters(120), 0));
        wayPoints.add(new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(-10)));

        List<TrajectoryConstraint> constraints = new ArrayList<>();
        constraints.add(new CentripetalAccelerationConstraint(kAngularAcceleration));
        constraints.add(new SwerveDriveConstraint(kKinematics, kVelocity, kAngularVelocity));

        TrajectoryConfig config = new TrajectoryConfig(
                kVelocity, kAcceleration, kAngularVelocity, kAngularAcceleration)
                .setReversed(false)
                .addConstraints(constraints);

        var start_orientation = Math.toRadians(-180.0);
        var end_orientation = Math.toRadians(0.0);

        List<QuinticHermiteSpline3D> splines = new ArrayList<>();

        var trajectory = SwerveTrajectoryGenerator.create_trajectory(
                wayPoints, start_orientation, end_orientation, config, splines);

        try (var writer = new FileWriter("c:\\Temp\\path.json", false)) {
            writer.write(TrajectoryUtil.serializeTrajectory(trajectory));
        }
    }

    @Test
    void test_generatespline() {
        List<Translation2d> wayPoints = new ArrayList<>();
        wayPoints.add(new Translation2d(0, 0));
        wayPoints.add(new Translation2d(Units.inchesToMeters(50), Units.inchesToMeters(50)));
        wayPoints.add(new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(10)));
        wayPoints.add(new Translation2d(Units.inchesToMeters(120), 0));
        wayPoints.add(new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(-10)));

//        List<TrajectoryConstraint> constraints = new ArrayList<>();
//        constraints.add(new CentripetalAccelerationConstraint(kAngularAcceleration));
//        constraints.add(new SwerveDriveConstraint(kKinematics, kVelocity, kAngularVelocity));

//        TrajectoryConfig config = new TrajectoryConfig(kVelocity, kAcceleration, kAngularVelocity, kAngularAcceleration)
//                .setReversed(false)
//                .addConstraints(constraints);

        var startOrientation = Math.toRadians(-180.0);
        var endOrientation = Math.toRadians(0.0);

        List<SwerveTrajectoryGenerator.TrajectoryPoint> points = createTrajectoryPoints(wayPoints);

        // Create an optimization parameter set per waypoint
        List<OptimizationParameters> optParams = new ArrayList<>();
        for (var i = 0; i < points.size(); i++) {
            optParams.add(new OptimizationParameters());
        }

//        elongation=0.500, re=0.150, rs=0.000, theta_elongation=1.000, theta_lambda=0.000,
//        elongation=0.500, re=0.300, rs=0.300, theta_elongation=1.000, theta_lambda=0.000,
//        elongation=0.500, re=0.000, rs=0.000, theta_elongation=1.000, theta_lambda=0.000,
//        elongation=0.500, re=0.150, rs=0.000, theta_elongation=1.000, theta_lambda=0.000,
//        elongation=0.500, re=0.000, rs=0.000, theta_elongation=1.000, theta_lambda=0.000,

        optParams.get(0).re=0.150;
        optParams.get(1).re=0.300;
        optParams.get(1).rs=0.300;
        optParams.get(3).re=0.150;

        var splines = generateSpline(points, startOrientation, endOrientation, optParams);
        for (var spline: splines) {
            System.out.println(spline.toString());
        }

        assertEquals(8, splines.size());
        // Start position
        assertEquals(wayPoints.get(0).x(), splines.get(0).xInitialControlVector[0]);
        assertEquals(wayPoints.get(0).y(), splines.get(0).yInitialControlVector[0]);

        // Start orientation
        assertEquals(startOrientation, splines.get(0).zInitialControlVector[0]);

        // End position
        assertEquals(wayPoints.get(4).x(), splines.get(7).xFinalControlVector[0]);
        assertEquals(wayPoints.get(4).y(), splines.get(7).yFinalControlVector[0]);

        // End orientation
        assertEquals(endOrientation, splines.get(7).zFinalControlVector[0]);
    }
}