package libraries.cheesylib.trajectory;

import frc.robot.config.RobotConfiguration;
import frc.robot.config.SwerveConfiguration;
import frc.robot.constants.Constants;
import frc.robot.paths.TrajectoryGenerator;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.util.Units;
import libraries.cheesylib.util.Util;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;


public class TrajectoryTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    public static final List<Translation2d> kWaypoints = Arrays.asList(
            new Translation2d(0.0, 0.0),
            new Translation2d(24.0, 0.0),
            new Translation2d(36.0, 12.0),
            new Translation2d(60.0, 12.0));

    @Test
    public void testConstruction() {
        // Empty constructor.
        Trajectory<Translation2d> traj = new Trajectory<>();
        assertTrue(traj.isEmpty());
        assertEquals(0.0, traj.getIndexView().first_interpolant(), kTestEpsilon);
        assertEquals(0.0, traj.getIndexView().last_interpolant(), kTestEpsilon);
        assertEquals(0, traj.length());

        // Set states at construction time.
        traj = new Trajectory<>(kWaypoints);
        assertFalse(traj.isEmpty());
        assertEquals(0.0, traj.getIndexView().first_interpolant(), kTestEpsilon);
        assertEquals(3.0, traj.getIndexView().last_interpolant(), kTestEpsilon);
        assertEquals(4, traj.length());
    }

    @Test
    public void testStateAccessors() {
        Trajectory<Translation2d> traj = new Trajectory<>(kWaypoints);

        assertEquals(kWaypoints.get(0), traj.getState(0));
        assertEquals(kWaypoints.get(1), traj.getState(1));
        assertEquals(kWaypoints.get(2), traj.getState(2));
        assertEquals(kWaypoints.get(3), traj.getState(3));

        assertEquals(kWaypoints.get(0), traj.getInterpolated(0.0).state());
        assertEquals(traj.getInterpolated(0.0).index_floor(), 0);
        assertEquals(traj.getInterpolated(0.0).index_ceil(), 0);
        assertEquals(kWaypoints.get(1), traj.getInterpolated(1.0).state());
        assertEquals(traj.getInterpolated(1.0).index_floor(), 1);
        assertEquals(traj.getInterpolated(1.0).index_ceil(), 1);
        assertEquals(kWaypoints.get(2), traj.getInterpolated(2.0).state());
        assertEquals(traj.getInterpolated(2.0).index_floor(), 2);
        assertEquals(traj.getInterpolated(2.0).index_ceil(), 2);
        assertEquals(kWaypoints.get(3), traj.getInterpolated(3.0).state());
        assertEquals(traj.getInterpolated(3.0).index_floor(), 3);
        assertEquals(traj.getInterpolated(3.0).index_ceil(), 3);

        assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), .25), traj.getInterpolated(0.25).state());
        assertEquals(traj.getInterpolated(0.25).index_floor(), 0);
        assertEquals(traj.getInterpolated(0.25).index_ceil(), 1);
        assertEquals(kWaypoints.get(1).interpolate(kWaypoints.get(2), .5), traj.getInterpolated(1.5).state());
        assertEquals(traj.getInterpolated(1.5).index_floor(), 1);
        assertEquals(traj.getInterpolated(1.5).index_ceil(), 2);
        assertEquals(kWaypoints.get(2).interpolate(kWaypoints.get(3), .75), traj.getInterpolated(2.75).state());
        assertEquals(traj.getInterpolated(2.75).index_floor(), 2);
        assertEquals(traj.getInterpolated(2.75).index_ceil(), 3);

        Trajectory<Translation2d>.IndexView index_view = traj.getIndexView();
        assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), .25), index_view.sample(0.25).state());
        assertEquals(kWaypoints.get(1).interpolate(kWaypoints.get(2), .5), index_view.sample(1.5).state());
        assertEquals(kWaypoints.get(2).interpolate(kWaypoints.get(3), .75), index_view.sample(2.75).state());
    }

    @Test
    public void StraightLineTest() {
        final RobotConfiguration mRobotConfiguration = RobotConfiguration.getRobotConfiguration(Constants.kRobot2022Name);
        final SwerveConfiguration mSwerveConfiguration = mRobotConfiguration.getSwerveConfiguration();

        var config = new TrajectoryConfig(
//                mSwerveConfiguration.trajectoryConfig.getMaxVelocity(),
                Units.feet_to_meters(14.2),
                mSwerveConfiguration.trajectoryConfig.getMaxAcceleration(),
                mSwerveConfiguration.trajectoryConfig.getMaxAngularVelocity(),
                mSwerveConfiguration.trajectoryConfig.getMaxAngularAcceleration())
                .setReversed(mSwerveConfiguration.trajectoryConfig.isReversed())
                .addConstraints(new ArrayList<>());


        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(Units.inches_to_meters(50), Units.inches_to_meters(0), Rotation2d.fromDegrees(0)));
        var trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);

        System.out.println("Trajectory");
        System.out.println(trajectory.toString());
    }
}
