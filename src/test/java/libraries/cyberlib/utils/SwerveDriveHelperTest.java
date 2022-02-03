package libraries.cyberlib.utils;

import frc.robot.Constants;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SwerveDriveHelperTest {

    @Test
    void calculateChassisSpeeds() {
        var chassisSpeeds = SwerveDriveHelper.calculateChassisSpeeds(
                0.0, 0.0, 0.0, false, false, false);
        assertEquals(0.0, chassisSpeeds.vxInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.vyInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.omegaInRadiansPerSecond);

        chassisSpeeds = SwerveDriveHelper.calculateChassisSpeeds(
                1.0, 0, 0, false, false, false);
        assertEquals(Constants.kSwerveDriveMaxSpeedInMetersPerSecond, chassisSpeeds.vxInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.vyInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.omegaInRadiansPerSecond);

        chassisSpeeds = SwerveDriveHelper.calculateChassisSpeeds(
                -1.0, 0, 0, false, false, false);
        assertEquals(-Constants.kSwerveDriveMaxSpeedInMetersPerSecond, chassisSpeeds.vxInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.vyInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.omegaInRadiansPerSecond);

        chassisSpeeds = SwerveDriveHelper.calculateChassisSpeeds(
                0.0, 1.0, 0, false, false, false);
        assertEquals(0.0, chassisSpeeds.vxInMetersPerSecond);
        assertEquals(Constants.kSwerveDriveMaxSpeedInMetersPerSecond, chassisSpeeds.vyInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.omegaInRadiansPerSecond);

        chassisSpeeds = SwerveDriveHelper.calculateChassisSpeeds(
                0.0, -1.0, 0, false, false, false);
        assertEquals(0.0, chassisSpeeds.vxInMetersPerSecond);
        assertEquals(-Constants.kSwerveDriveMaxSpeedInMetersPerSecond, chassisSpeeds.vyInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.omegaInRadiansPerSecond);

        chassisSpeeds = SwerveDriveHelper.calculateChassisSpeeds(
                0.0, 0.0, 1.0, false, false, false);
        assertEquals(0.0, chassisSpeeds.vxInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.vyInMetersPerSecond);

        var maxSpeed = Constants.kSwerveRotationMaxSpeedInRadiansPerSecond * 0.8; //SwerveDriveHelper's kHighPowerRotationScalar
        assertEquals(maxSpeed, chassisSpeeds.omegaInRadiansPerSecond);

        chassisSpeeds = SwerveDriveHelper.calculateChassisSpeeds(
                0.0, 0.0, 1.0, true, false, false);
        assertEquals(0.0, chassisSpeeds.vxInMetersPerSecond);
        assertEquals(0.0, chassisSpeeds.vyInMetersPerSecond);

        maxSpeed = Constants.kSwerveRotationMaxSpeedInRadiansPerSecond * 0.5; //SwerveDriveHelper's kHighPowerRotationScalar
        assertEquals(maxSpeed, chassisSpeeds.omegaInRadiansPerSecond);
    }
}