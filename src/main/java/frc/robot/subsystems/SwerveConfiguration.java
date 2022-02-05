package frc.robot.subsystems;

import libraries.cheesylib.geometry.Translation2d;

import java.util.Arrays;
import java.util.List;

public class SwerveConfiguration {
    public final double wheelbaseLengthInMeters;
    public final double wheelbaseWidthInMeters;
    public final double maxSpeedInMetersPerSecond;
    public final double maxSpeedInRadiansPerSecond;
    public final List<Translation2d> moduleLocations;

    /**
     * Creates an instance of SwerveConfiguration
     * <p>
     * @param wheelbaseLengthInMeters Length in meters of distance between front and back wheels
     * @param wheelbaseWidthInMeters Length in meters of distance between left and right wheels
     * @param maxSpeedInMetersPerSecond Max translation speed in meters
     * @param maxSpeedInRadiansPerSecondLimit Max rotation speed in radians per second.  Note translation speed
     *                                        and wheelbase geometry may limit rotations speed.  If set 0.0, then
     *                                        translation speed and wheelbase geometry determines max rotation speed.
     */
    public SwerveConfiguration (double wheelbaseLengthInMeters,
                                double wheelbaseWidthInMeters,
                                double maxSpeedInMetersPerSecond,
                                double maxSpeedInRadiansPerSecondLimit) {

        this.wheelbaseLengthInMeters = wheelbaseLengthInMeters;
        this.wheelbaseWidthInMeters = wheelbaseWidthInMeters;
        this.maxSpeedInMetersPerSecond = maxSpeedInMetersPerSecond;
        var radiusInMeters =
                Math.hypot(this.wheelbaseLengthInMeters / 2, this.wheelbaseWidthInMeters / 2);
        this.maxSpeedInRadiansPerSecond = maxSpeedInRadiansPerSecondLimit > 0.0 ?
                Math.min((this.maxSpeedInMetersPerSecond / radiusInMeters), maxSpeedInRadiansPerSecondLimit) :
                (this.maxSpeedInMetersPerSecond / radiusInMeters);

        // If CW, then right is positive.  If CCW left is positive
        Translation2d kFrontRightModuleLocation = new Translation2d(wheelbaseLengthInMeters / 2, wheelbaseWidthInMeters / 2);
        Translation2d kFrontLeftModuleLocation = new Translation2d(wheelbaseLengthInMeters / 2, -wheelbaseWidthInMeters / 2);
        Translation2d kBackLeftModuleLocation = new Translation2d(-wheelbaseLengthInMeters / 2, -wheelbaseWidthInMeters / 2);
        Translation2d kBackRightModuleLocation = new Translation2d(-wheelbaseLengthInMeters / 2, wheelbaseWidthInMeters / 2);

        moduleLocations = Arrays.asList(
                kFrontRightModuleLocation, kFrontLeftModuleLocation, kBackLeftModuleLocation, kBackRightModuleLocation);

    }
}
