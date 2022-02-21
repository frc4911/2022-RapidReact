package frc.robot.sensors;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.Constants;
import frc.robot.Ports;
import libraries.cheesylib.geometry.Rotation2d;

public class PigeonTwo implements IMU {

    private static PigeonTwo instance = null;

    private Pigeon2 pigeon;

    private PigeonTwo() {
        try {
            pigeon = new Pigeon2(Ports.PIGEON);
        } catch (Exception e) {
            System.out.println(e);
        }
    }

    public static PigeonTwo getInstance() {
        if (instance == null) {
            instance = new PigeonTwo();
        }
        return instance;
    }

    @Override
    public boolean isGood() {
        // Unlike the original Pigeon, the Pigeon2 is always ready
        return true;
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    @Override
    public double getPitch() {
        return pigeon.getPitch();
    }

    @Override
    public double getRoll() {
        return pigeon.getRoll();
    }

    @Override
    public double[] getYPR() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr;
    }

    @Override
    public void setAngle(double angle) {
        pigeon.setYaw(-angle, Constants.kLongCANTimeoutMs);
        System.out.println("Pigeon angle set to: " + angle);
    }

    @Override
    public void outputToSmartDashboard() {
        // no-op
    }

}