package libraries.cyberlib.trajectory;

import edu.wpi.first.math.util.Units;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cyberlib.kinematics.SwerveDriveKinematics;
import libraries.cyberlib.spline.QuinticHermiteSpline3D;
import libraries.cyberlib.trajectory.constraints.CentripetalAccelerationConstraint;
import libraries.cyberlib.trajectory.constraints.SwerveDriveConstraint;
import libraries.cyberlib.trajectory.constraints.TrajectoryConstraint;
import libraries.cyberlib.trajectory.swerve.SwerveTrajectoryGenerator;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class TrajectoryJsonTest {
    final static double kRobotWidth = Units.inchesToMeters(30.0 / 12.0);
    final static double kRobotLength = Units.inchesToMeters(30.0 / 12.0);
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
    void deserializeMatches() {
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

        var deserialized =
                assertDoesNotThrow(
                        () ->
                                TrajectoryUtil.deserializeTrajectory(
                                        TrajectoryUtil.serializeTrajectory(trajectory)));

        assertEquals(trajectory.getStates(), deserialized.getStates());

//        try {
//            var json = TrajectoryUtil.serializeTrajectory(trajectory);
//            System.out.println(json);
//        } catch (JsonProcessingException e) {
//            e.printStackTrace();
//        }
    }
}
