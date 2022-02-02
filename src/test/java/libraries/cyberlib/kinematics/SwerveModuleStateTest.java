package libraries.cyberlib.kinematics;

import libraries.cheesylib.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SwerveModuleStateTest {
    private static final double kEpsilon = 1E-9;

    @Test
    void testOptimize() {
        var angleA = Rotation2d.fromDegrees(45);
        var refA = new SwerveModuleState(-2.0, Rotation2d.fromDegrees(180));
        var optimizedA = SwerveModuleState.optimize(refA, angleA);

        assertAll(
                () -> assertEquals(2.0, optimizedA.speedInMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, optimizedA.angle.getDegrees(), kEpsilon));

        var angleB = Rotation2d.fromDegrees(-50);
        var refB = new SwerveModuleState(4.7, Rotation2d.fromDegrees(41));
        var optimizedB = SwerveModuleState.optimize(refB, angleB);

        assertAll(
                () -> assertEquals(-4.7, optimizedB.speedInMetersPerSecond, kEpsilon),
                () -> assertEquals(-139.0, optimizedB.angle.getDegrees(), kEpsilon));
    }

    @Test
    void testNoOptimize() {
        var angleA = Rotation2d.fromDegrees(0);
        var refA = new SwerveModuleState(2.0, Rotation2d.fromDegrees(89));
        var optimizedA = SwerveModuleState.optimize(refA, angleA);

        assertAll(
                () -> assertEquals(2.0, optimizedA.speedInMetersPerSecond, kEpsilon),
                () -> assertEquals(89.0, optimizedA.angle.getDegrees(), kEpsilon));

        var angleB = Rotation2d.fromDegrees(0);
        var refB = new SwerveModuleState(-2.0, Rotation2d.fromDegrees(-2));
        var optimizedB = SwerveModuleState.optimize(refB, angleB);

        assertAll(
                () -> assertEquals(-2.0, optimizedB.speedInMetersPerSecond, kEpsilon),
                () -> assertEquals(-2.0, optimizedB.angle.getDegrees(), kEpsilon));
    }
}