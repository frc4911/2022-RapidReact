package frc.robot.sensors;

import libraries.cheesylib.geometry.Rotation2d;

/**
 * This interface is intended to abstract away the implementation detail of
 * which IMU a particular robot has on it. This is currently written
 * specifically to the Pigeon's as that's what we're using today, but could be
 * genericized if needed in the future.
 */
public interface IMU {
    public boolean isGood();

    /**
     * Represents the rotation around the vertical axis (turning left/right),
     * increasing in the CCW direction
     * 
     * @return Rotation2d vector representing the sine and cosine of the current
     *         rotation on the unit circle
     */
    public Rotation2d getYaw();

    /**
     * Represents the rotation around the transverse axis (up/down hill)
     * 
     * @return Pitch in degrees of the IMU
     */
    public double getPitch();

    /**
     * Represents the rotation around the longitudinal axis (side to side
     * 
     * @return Roll in degrees of the IMU
     */
    public double getRoll();

    /**
     * Represents the yaw, pitch, and roll currently reported by the IMU
     * 
     * @return 3-item array of the yaw, pitch, and roll in degrees
     */
    public double[] getYPR();

    /**
     * Used to set the starting angle of the IMU based on where it is, or to
     * reset to a known good state
     * 
     * @param angleInDegrees starting angle of the IMU in degrees
     */
    public void setAngle(double angleInDegrees);

    /**
     * Helper method to allow for the IMU to log an desired values to
     * SmartDashboard. Wherever this is implemented needs to handle
     * getting the SmartDashboard and then writing values to it.
     */
    public void outputToSmartDashboard();

    public enum ImuType {
        PIGEON,
        PIGEON2
    }

    /**
     * Helper method to convert the configuration ImuType to a working
     * instance of the specific IMU version in use
     * 
     * @param imuType The desired ImuType
     * @return The specific instance of the IMU on the robot
     */
    public static IMU createImu(ImuType imuType) {
        switch (imuType) {
            case PIGEON:
                return Pigeon.getInstance();
            default:
            case PIGEON2:
                return PigeonTwo.getInstance();
        }
    }
}
