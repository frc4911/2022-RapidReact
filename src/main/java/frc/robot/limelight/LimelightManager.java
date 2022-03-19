package frc.robot.limelight;

import frc.robot.subsystems.Limelight;

public class LimelightManager {
    private static String sClassName;
    private static int sInstanceCount;

    private Limelight limelight;

    private static LimelightManager mInstance;

    public static LimelightManager getInstance() {
        if (mInstance == null) {
            mInstance = new LimelightManager();
        }
        return mInstance;
    }

    private LimelightManager() {
    }

    public void setLimelight(Limelight limelight) {
        this.limelight = limelight;
    }

    public Limelight getLimelight() {
        return limelight;
    }

    public void writePeriodicOutputs() {
        limelight.writePeriodicOutputs();
    }

}
