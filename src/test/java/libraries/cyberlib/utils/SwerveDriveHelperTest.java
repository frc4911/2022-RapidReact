package libraries.cyberlib.utils;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SwerveDriveHelperTest {
    private static final double SWERVE_DRIVE_MAX_INPUT = 1.0;
    private static final double SWERVE_ROTATION_MAX_INPUT = 1.0;

    @Test
    void calculatedriveSignal() {

        var driveSignal = SwerveDriveHelper.calculate(
                0.0, 0.0, 0.0, false, false, false);
        assertEquals(0.0, driveSignal.getTranslation().x());
        assertEquals(0.0, driveSignal.getTranslation().y());
        assertEquals(0.0, driveSignal.getRotation());

        driveSignal = SwerveDriveHelper.calculate(
                1.0, 0, 0, false, false, false);
        assertEquals(SWERVE_DRIVE_MAX_INPUT, driveSignal.getTranslation().x());
        assertEquals(0.0, driveSignal.getTranslation().y());
        assertEquals(0.0, driveSignal.getRotation());

        driveSignal = SwerveDriveHelper.calculate(
                -1.0, 0, 0, false, false, false);
        assertEquals(-SWERVE_DRIVE_MAX_INPUT, driveSignal.getTranslation().x());
        assertEquals(0.0, driveSignal.getTranslation().y());
        assertEquals(0.0, driveSignal.getRotation());

        driveSignal = SwerveDriveHelper.calculate(
                0.0, 1.0, 0, false, false, false);
        assertEquals(0.0, driveSignal.getTranslation().x());
        assertEquals(SWERVE_DRIVE_MAX_INPUT, driveSignal.getTranslation().y());
        assertEquals(0.0, driveSignal.getRotation());

        driveSignal = SwerveDriveHelper.calculate(
                0.0, -1.0, 0, false, false, false);
        assertEquals(0.0, driveSignal.getTranslation().x());
        assertEquals(-SWERVE_DRIVE_MAX_INPUT, driveSignal.getTranslation().y());
        assertEquals(0.0, driveSignal.getRotation());

        driveSignal = SwerveDriveHelper.calculate(
                0.0, 0.0, 1.0, false, false, false);
        assertEquals(0.0, driveSignal.getTranslation().x());
        assertEquals(0.0, driveSignal.getTranslation().y());

        var maxSpeed = SWERVE_ROTATION_MAX_INPUT * 0.8; //SwerveDriveHelper's kHighPowerRotationScalar
        assertEquals(maxSpeed, driveSignal.getRotation());

        driveSignal = SwerveDriveHelper.calculate(
                0.0, 0.0, 1.0, true, false, false);
        assertEquals(0.0, driveSignal.getTranslation().x());
        assertEquals(0.0, driveSignal.getTranslation().y());

        maxSpeed = SWERVE_ROTATION_MAX_INPUT * 0.5; //SwerveDriveHelper's kHighPowerRotationScalar
        assertEquals(maxSpeed, driveSignal.getRotation());
    }

//    @Test
//    public void ramp() {
//        for (double i = 0.0; i < 1.05; i += 0.05) {
//            var driveSignal = SwerveDriveHelper.calculate(i,  0.0, 0.0, false, false, false);
//            System.out.format("%.2f - %s\n", i, driveSignal);
//        }
//        for (double i = 0.0; i < 1.05; i += 0.05) {
//            var driveSignal = SwerveDriveHelper.calculate(i,  i, 0.6, false, false, false);
//            System.out.format("%.2f - %s\n", i, driveSignal);
//        }
//
//    }

}