package libraries.cyberlib.utils;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SwerveDriveHelperTest {
    private static final double SWERVE_DRIVE_MAX_SPEED_IN_METERS_PER_SECOND = 5.0;
    private static final double SWERVE_ROTATION_MAX_SPEED_IN_RADIANS_PER_SECOND = Math.toRadians(270);

    @Test
    void calculateChassisSpeeds() {
        SwerveDriveHelper swerveDriveHelper = new SwerveDriveHelper(
                SWERVE_DRIVE_MAX_SPEED_IN_METERS_PER_SECOND, SWERVE_ROTATION_MAX_SPEED_IN_RADIANS_PER_SECOND);

        var chassisSpeeds = swerveDriveHelper.calculateChassisSpeeds(
                0.0, 0.0, 0.0, false, false, false);
        assertEquals(0.0, chassisSpeeds.vxInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.vyInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.omegaInRadiansPerSecond);

        chassisSpeeds = swerveDriveHelper.calculateChassisSpeeds(
                1.0, 0, 0, false, false, false);
        assertEquals(SWERVE_DRIVE_MAX_SPEED_IN_METERS_PER_SECOND, chassisSpeeds.vxInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.vyInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.omegaInRadiansPerSecond);

        chassisSpeeds = swerveDriveHelper.calculateChassisSpeeds(
                -1.0, 0, 0, false, false, false);
        assertEquals(-SWERVE_DRIVE_MAX_SPEED_IN_METERS_PER_SECOND, chassisSpeeds.vxInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.vyInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.omegaInRadiansPerSecond);

        chassisSpeeds = swerveDriveHelper.calculateChassisSpeeds(
                0.0, 1.0, 0, false, false, false);
        assertEquals(0.0, chassisSpeeds.vxInMetersPerSecond);
        assertEquals(SWERVE_DRIVE_MAX_SPEED_IN_METERS_PER_SECOND, chassisSpeeds.vyInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.omegaInRadiansPerSecond);

        chassisSpeeds = swerveDriveHelper.calculateChassisSpeeds(
                0.0, -1.0, 0, false, false, false);
        assertEquals(0.0, chassisSpeeds.vxInMetersPerSecond);
        assertEquals(-SWERVE_DRIVE_MAX_SPEED_IN_METERS_PER_SECOND, chassisSpeeds.vyInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.omegaInRadiansPerSecond);

        chassisSpeeds = swerveDriveHelper.calculateChassisSpeeds(
                0.0, 0.0, 1.0, false, false, false);
        assertEquals(0.0, chassisSpeeds.vxInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.vyInMetersPerSecond);

        var maxSpeed = SWERVE_ROTATION_MAX_SPEED_IN_RADIANS_PER_SECOND * 0.8; //SwerveDriveHelper's kHighPowerRotationScalar
        assertEquals(maxSpeed, chassisSpeeds.omegaInRadiansPerSecond);

        chassisSpeeds = swerveDriveHelper.calculateChassisSpeeds(
                0.0, 0.0, 1.0, true, false, false);
        assertEquals(0.0, chassisSpeeds.vxInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.vyInMetersPerSecond);

        maxSpeed = SWERVE_ROTATION_MAX_SPEED_IN_RADIANS_PER_SECOND * 0.5; //SwerveDriveHelper's kHighPowerRotationScalar
        assertEquals(maxSpeed, chassisSpeeds.omegaInRadiansPerSecond);
    }
}