package libraries.cyberlib.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

import java.util.List;

import static libraries.cyberlib.trajectory.TrajectoryGenerator.generateTrajectory;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class TrajectoryConcatenateTest {
    @Test
    void testStates() {
        var t1 =
                generateTrajectory(
                        new Pose2d(),
                        List.of(),
                        new Pose2d(1, 1, new Rotation2d()),
                        new TrajectoryConfig(2, 2, 0, 0));

        var t2 =
                generateTrajectory(
                        new Pose2d(1, 1, new Rotation2d()),
                        List.of(),
                        new Pose2d(2, 2, Rotation2d.fromDegrees(45)),
                        new TrajectoryConfig(2, 2));

        var t = t1.concatenate(t2);

        double time = -1.0;
        for (int i = 0; i < t.getStates().size(); ++i) {
            var state = t.getStates().get(i);

            // Make sure that the timestamps are strictly increasing.
            assertTrue(state.timeSeconds > time);
            time = state.timeSeconds;

            // Ensure that the states in t are the same as those in t1 and t2.
            if (i < t1.getStates().size()) {
                assertEquals(state, t1.getStates().get(i));
            } else {
                var st = t2.getStates().get(i - t1.getStates().size() + 1);
                st.timeSeconds += t1.getTotalTimeSeconds();
                assertEquals(state, st);
            }
        }
    }
}
