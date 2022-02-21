package frc.robot.subsystems;

import libraries.cheesylib.geometry.Rotation2d;

/**
 * This interface is intended to abstract away the implementation detail of
 * which IMU a particular robot has on it. This is currently written
 * specifically to the Pigeon's as that's what we're using today, but could be
 * genericized if needed in the future.
 */
public interface IMU {
    public boolean isGood();

    public Rotation2d getYaw();

    public double getPitch();

    public double getRoll();

    public double[] getYPR();

    public void setAngle(double angle);

    public void outputToSmartDashboard();

    public enum ImuType {
        PIGEON,
        PIGEON2
    }

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
