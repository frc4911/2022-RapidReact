package libraries.cyberlib.kinematics;

import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import org.ejml.simple.SimpleMatrix;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
/**
 * Helper class that converts a chassis velocity (dx, dy, and dtheta components)
 * into individual module states (speed and angle).
 *
 * <p>The inverse kinematics (converting from a desired chassis velocity to
 * individual module states) uses the relative locations of the modules with
 * respect to the center of rotation. The center of rotation for inverse
 * kinematics is also variable. This means that you can set your set your center
 * of rotation in a corner of the robot to perform special evasion maneuvers.
 *
 * <p>Forward kinematics (converting an array of module states into the overall
 * chassis motion) is performs the exact opposite of what inverse kinematics
 * does. Since this is an overdetermined system (more equations than variables),
 * we use a least-squares approximation.
 *
 * <p>The inverse kinematics: [moduleStates] = [moduleLocations] * [chassisSpeeds]
 * We take the Moore-Penrose pseudoinverse of [moduleLocations] and then
 * multiply by [moduleStates] to get our chassis speeds.
 *
 * <p>Forward kinematics is also used for odometry -- determining the position of
 * the robot on the field using encoders and a gyro.
 */
public class SwerveDriveKinematics {
    private final SimpleMatrix mInverseKinematics;
    private final SimpleMatrix mForwardKinematics;

    private final int mNumModules;
    private final Translation2d[] mModules;
    private Translation2d mPrevCoR = new Translation2d();

    /**
     * Constructs a swerve drive kinematics object. This takes in a variable
     * number of wheel locations as Translation2ds. The order in which you pass in
     * the wheel locations is the same order that you will receive the module
     * states when performing inverse kinematics. It is also expected that you
     * pass in the module states in the same order when calling the forward
     * kinematics methods.
     *
     * @param wheelsLocationsInMeters The locations of the wheels relative to the physical center
     *                     of the robot as meters.
    */
    public SwerveDriveKinematics(List<Translation2d> wheelsLocationsInMeters) {
        this(wheelsLocationsInMeters.toArray(Translation2d[]::new));
    }

    public SwerveDriveKinematics(Translation2d... wheelsLocationsInMeters) {
        if (wheelsLocationsInMeters.length < 2) {
            throw new IllegalArgumentException("A swerve drive requires at least two modules");
        }

        mNumModules = wheelsLocationsInMeters.length;
        mModules = Arrays.copyOf(wheelsLocationsInMeters, mNumModules);
        mInverseKinematics = new SimpleMatrix(mNumModules * 2, 3);

        for (int i = 0; i < mNumModules; i++) {
            mInverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -mModules[i].y());
            mInverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +mModules[i].x());
        }

        mForwardKinematics = mInverseKinematics.pseudoInverse();
    }

    /**
     * Performs inverse kinematics to return the module states from a desired chassis velocity. This
     * method is often used to convert joystick values into module speeds and angles.
     *
     * <p>This function also supports variable centers of rotation. During normal operations, the
     * center of rotation is usually the same as the physical center of the robot; therefore, the
     * argument is defaulted to that use case. However, if you wish to change the center of rotation
     * for evasive maneuvers, vision alignment, or for any other use case, you can do so.
     *
     * @param chassisSpeeds    The desired chassis speed.
     * @param centerOfRotationInMeters The center of rotation. For example, if you set the
     *                         center of rotation at one corner of the robot and provide
     *                         a chassis speed that only has a dtheta component, the robot
     *                         will rotate around that corner.
     * @return An array containing the module states. Use caution because these module states are not
     *     normalized. Sometimes, a user input may cause one of the module speeds to go above the
     *     attainable max velocity. Use the {@link #desaturateWheelSpeeds(SwerveModuleState[], double)
     *     DesaturateWheelSpeeds} function to rectify this issue.
     */
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds,
                                                    Translation2d centerOfRotationInMeters) {
        if (!centerOfRotationInMeters.equals(mPrevCoR)) {
            for (int i = 0; i < mNumModules; i++) {
                mInverseKinematics.setRow(
                        i * 2 + 0,
                        0, /* Start Data */
                        1,
                        0,
                        -mModules[i].y() + centerOfRotationInMeters.y());
                mInverseKinematics.setRow(
                        i * 2 + 1,
                        0, /* Start Data */
                        0,
                        1,
                        +mModules[i].x() - centerOfRotationInMeters.x());
            }
            mPrevCoR = centerOfRotationInMeters;
        }

        var chassisSpeedsVector = new SimpleMatrix(3, 1);
        chassisSpeedsVector.setColumn(
                0,
                0,
                chassisSpeeds.vxInMetersPerSecond,
                chassisSpeeds.vyInMetersPerSecond,
                chassisSpeeds.omegaInRadiansPerSecond);

        var moduleStatesMatrix = mInverseKinematics.mult(chassisSpeedsVector);
        SwerveModuleState[] moduleStates = new SwerveModuleState[mNumModules];

        for (int i = 0; i < mNumModules; i++) {
            double x = moduleStatesMatrix.get(i * 2, 0);
            double y = moduleStatesMatrix.get(i * 2 + 1, 0);

            double speed = Math.hypot(x, y);
            Rotation2d angle = new Rotation2d(x, y, true);

            moduleStates[i] = new SwerveModuleState(speed, angle);
        }
        // brian temp debug
        // if(++throttlePrints%printFreq==0){
        //     System.out.println("00 sdk toSwerveModuleStates (moduleStates[0]) ("+moduleStates[0].toString()+")");
        //     System.out.println("00 sdk toSwerveModuleStates (moduleStates[1]) ("+moduleStates[1].toString()+")");
        //     System.out.println("00 sdk toSwerveModuleStates (moduleStates[2]) ("+moduleStates[2].toString()+")");
        //     System.out.println("00 sdk toSwerveModuleStates (moduleStates[3]) ("+moduleStates[3].toString()+")");
        // }
        return moduleStates;
    }

    
    // brian temp debug
    int throttlePrints = 0;
    final int printFreq = 10;
    final double[] fixedAngles = {Math.toRadians(45),Math.toRadians(135),Math.toRadians(-135),Math.toRadians(-45)};

    // brian the rntire method is temp debug
    public SwerveModuleState[] toSwerveModuleStates2(ChassisSpeeds chassisSpeeds, Translation2d unused) {

        double xVel = chassisSpeeds.vxInMetersPerSecond;
        double yVel = chassisSpeeds.vyInMetersPerSecond;
        double rot = chassisSpeeds.omegaInRadiansPerSecond;

        SwerveModuleState[] moduleStates = new SwerveModuleState[mNumModules];

        for (int i = 0; i < mNumModules; i++) {
            
            double xV = xVel+Math.cos(fixedAngles[i])*rot;
            double yV = yVel+Math.sin(fixedAngles[i])*rot;
            moduleStates[i] = new SwerveModuleState(Math.hypot(xV, yV), new Rotation2d(xV,yV,true));
        }

        // if(++throttlePrints%printFreq==0){
        //     System.out.println("00 sdk toSwerveModuleStates (moduleStates[0]) ("+moduleStates[0].toString()+")");
        //     System.out.println("00 sdk toSwerveModuleStates (moduleStates[1]) ("+moduleStates[1].toString()+")");
        //     System.out.println("00 sdk toSwerveModuleStates (moduleStates[2]) ("+moduleStates[2].toString()+")");
        //     System.out.println("00 sdk toSwerveModuleStates (moduleStates[3]) ("+moduleStates[3].toString()+")");
        // }
        return moduleStates;
    }

    /**
     * Performs inverse kinematics. See {@link #toSwerveModuleStates(ChassisSpeeds, Translation2d)}
     * toSwerveModuleStates for more information.
     *
     * @param chassisSpeeds The desired chassis speed.
     * @return An array containing the module states.
     */
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        return toSwerveModuleStates(chassisSpeeds, new Translation2d());
    }

    /**
     * Performs forward kinematics to return the resulting chassis state from the
     * given module states. This method is often used for odometry -- determining
     * the robot's position on the field using data from the real-world speed and
     * angle of each module on the robot.
     *
     * @param wheelStates The state of the modules (as a SwerveModuleState type)
     *                    as measured from respective encoders and gyros. The order of the swerve
     *                    module states should be same as passed into the constructor of this class.
     * @return The resulting chassis speed.
     */

    public ChassisSpeeds toChassisSpeeds(SwerveModuleState... wheelStates) {
        if (wheelStates.length != mNumModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor"
            );
        }

        var moduleStatesMatrix = new SimpleMatrix(mNumModules * 2, 1);

        for (int i = 0; i < mNumModules; i++) {
            var module = wheelStates[i];
            moduleStatesMatrix.set(i * 2, 0, module.speedInMetersPerSecond * module.angle.cos());
            moduleStatesMatrix.set(i * 2 + 1, module.speedInMetersPerSecond * module.angle.sin());
        }

        var chassisSpeedsVector = mForwardKinematics.mult(moduleStatesMatrix);
        return new ChassisSpeeds(
                chassisSpeedsVector.get(0, 0),
                chassisSpeedsVector.get(1, 0),
                chassisSpeedsVector.get(2, 0));
    }

    /**
     * Renormalizes the wheel speeds if any individual speed is above the specified maximum.
     *
     * <p>Sometimes, after inverse kinematics, the requested speed from one or more modules may be
     * above the max attainable speed for the driving motor on that module. To fix this issue, one can
     * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
     * absolute threshold, while maintaining the ratio of speeds between modules.
     *
     * @param moduleStates       Reference to array of module states. The array will be
     *                           mutated with the normalized speeds!
     * @param attainableMaxSpeedInMetersPerSecond The absolute max speed that a module can reach.
     */
    public static void desaturateWheelSpeeds(SwerveModuleState[] moduleStates,
                                            double attainableMaxSpeedInMetersPerSecond) {
        double realMaxSpeed = Collections.max(Arrays.asList(moduleStates)).speedInMetersPerSecond;
        if (realMaxSpeed > attainableMaxSpeedInMetersPerSecond) {
            for (SwerveModuleState moduleState : moduleStates) {
                moduleState.speedInMetersPerSecond = moduleState.speedInMetersPerSecond / realMaxSpeed
                        * attainableMaxSpeedInMetersPerSecond;
            }
        }
    }
 }
