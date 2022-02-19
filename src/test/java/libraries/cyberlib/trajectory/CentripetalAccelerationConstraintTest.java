package libraries.cyberlib.trajectory;

import edu.wpi.first.math.util.Units;
import libraries.cyberlib.trajectory.constraints.CentripetalAccelerationConstraint;
import org.junit.jupiter.api.Test;

import java.util.Collections;

import static libraries.cheesylib.util.Util.kEpsilon;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class CentripetalAccelerationConstraintTest {
    @SuppressWarnings("LocalVariableName")
    @Test
    void testCentripetalAccelerationConstraint() {
        double maxCentripetalAcceleration = Units.feetToMeters(7.0); // 7 feet per second squared
        var constraint = new CentripetalAccelerationConstraint(maxCentripetalAcceleration);

        Trajectory trajectory =
                TrajectoryGeneratorTest.getTrajectory(Collections.singletonList(constraint));

        var duration = trajectory.getTotalTimeSeconds();
        var t = 0.0;
        var dt = 0.02;

        while (t < duration) {
            var point = trajectory.sample(t);
            var centripetalAcceleration
                    = Math.pow(point.velocityMetersPerSecond, 2) * point.curvatureRadPerMeter;

            // Mirror logic
            if (centripetalAcceleration < kEpsilon) {
                break;
            }

            t += dt;
            assertTrue(centripetalAcceleration <= maxCentripetalAcceleration + 0.05);
        }
    }
}
