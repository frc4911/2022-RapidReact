package libraries.cyberlib.trajectory;

import static edu.wpi.first.math.util.Units.feetToMeters;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;


class WPILIBTragectoryGeneratorTest {
    static Trajectory getTrajectory(
            List<? extends TrajectoryConstraint> constraints) {
        final double maxVelocity = feetToMeters(12.0);
        final double maxAccel = feetToMeters(12);

        // 2018 cross scale auto waypoints.
        var sideStart =
                new Pose2d(feetToMeters(1.54), feetToMeters(23.23), Rotation2d.fromDegrees(-180));
        var crossScale =
                new Pose2d(feetToMeters(23.7), feetToMeters(6.8), Rotation2d.fromDegrees(-160));

        var waypoints = new ArrayList<Pose2d>();
        waypoints.add(sideStart);
        waypoints.add(
                sideStart.plus(
                        new Transform2d(
                                new Translation2d(feetToMeters(-13), feetToMeters(0)), new Rotation2d())));
        waypoints.add(
                sideStart.plus(
                        new Transform2d(
                                new Translation2d(feetToMeters(-19.5), feetToMeters(5)),
                                Rotation2d.fromDegrees(-90))));
        waypoints.add(crossScale);

        TrajectoryConfig config =
                new TrajectoryConfig(maxVelocity,maxAccel)
                    .setReversed(true)
                    .addConstraints(constraints);

        return TrajectoryGenerator.generateTrajectory(waypoints, config);
    }

    @Test
    @SuppressWarnings("LocalVariableName")
    void testGenerationAndConstraints() {
        Trajectory trajectory = getTrajectory(new ArrayList<>());
        System.out.println(trajectory.toString());
        System.out.println();

        double duration = trajectory.getTotalTimeSeconds();
        double t = 0.0;
        double dt = 0.02;

        while (t < duration) {
            var point = trajectory.sample(t);
            System.out.println(point.toString());
            t += dt;
            assertAll(
                    () -> assertTrue(Math.abs(point.velocityMetersPerSecond) < feetToMeters(12.0) + 0.05),
                    () ->assertTrue(Math.abs(point.accelerationMetersPerSecondSq) < feetToMeters(12.0) + 0.05));
        }
    }

    @Test
    void testMalformedTrajectory() {
        var traj =
                TrajectoryGenerator.generateTrajectory(
                        Arrays.asList(
                                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                                new Pose2d(1, 0, Rotation2d.fromDegrees(180))),
                        new TrajectoryConfig(feetToMeters(12), feetToMeters(12)));

        assertEquals(traj.getStates().size(), 1);
        assertEquals(traj.getTotalTimeSeconds(), 0);
    }
}
