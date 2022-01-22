package libraries.cyberlib.kinematics;


import libraries.cheesylib.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;


class ChassisSpeedsTest {
  private static final double kEpsilon = 1E-9;

  @Test
  void testFieldRelativeConstruction() {
    final var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        1.0, 0.0, 0.5, Rotation2d.fromDegrees(-90.0)
    );

    assertAll(
        () -> assertEquals(0.0, chassisSpeeds.vxInMetersPerSecond, kEpsilon),
        () -> assertEquals(1.0, chassisSpeeds.vyInMetersPerSecond, kEpsilon),
        () -> assertEquals(0.5, chassisSpeeds.omegaInRadiansPerSecond, kEpsilon)
    );
  }
}
