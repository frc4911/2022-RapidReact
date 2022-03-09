package frc.robot.subsystems.LimeLights;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.config.LimelightConfig;
import frc.robot.config.PipelineConfiguration;
import frc.robot.constants.Constants;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.util.Util;
import libraries.cheesylib.vision.TargetInfo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

/**
 * Subsystem for interacting with Limelight 2+.
 *
 * <p>The Limelight is used to detect targets and feed them to a GoalTracker in RobotState</p>
 */
public class Limelight extends Subsystem {

    private static final Comparator<Translation2d> xSort = Comparator.comparingDouble(Translation2d::x);
    private static final Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::y);
    private final NetworkTable mNetworkTable;
    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private final double[] mZeroArray = new double[]{0, 0, 0, 0, 0, 0, 0, 0};
    private final List<TargetInfo> mTargets = new ArrayList<>();
    private final boolean mLoggingEnabled = true;
    private LimelightConfig mConfig;
    private PipelineConfiguration mPipelineConfig;
    private boolean mOutputsHaveChanged = true;
    private boolean mSeesTarget = false;
    private double mLastStart;
    private int mDefaultSchedDelta = 20;
    private String sClassName;

    public Limelight(LimelightConfig limelight2Config, PipelineConfiguration pipelineConfig) {
        sClassName = "Limelight2";
        mConfig = limelight2Config;
        setPipelineConfig(pipelineConfig);
        mNetworkTable = NetworkTableInstance.getDefault().getTable(limelight2Config.getTableName());
    }

    public static List<TargetInfo> getRawTargetInfos(List<double[]> corners, PipelineConfiguration pipeline, List<TargetInfo> targets,
                                                     double kHorizontalFOV, double kVerticalFOV) {
        if (corners == null) {
            return null;
        }

        List<double[]> transformedCorners = new ArrayList<>(2);
        for (int i = 0; i < 2; i++) {
            double[] corner = corners.get(i);
            corner = pipeline.normalize(corner);
            transformedCorners.add(corner);
        }

        double slope = 0.0;
        if (Math.abs(transformedCorners.get(1)[0] - transformedCorners.get(0)[0]) > Util.kEpsilon) {
            slope = (transformedCorners.get(1)[1] - transformedCorners.get(0)[1]) /
                    (transformedCorners.get(1)[0] - transformedCorners.get(0)[0]);
        }

        targets.clear();

        double VPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
        double VPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
        for (int i = 0; i < 2; ++i) {
            // Average of y and z;
            double y_pixels = transformedCorners.get(i)[0];
            double z_pixels = transformedCorners.get(i)[1];

            // Redefine to robot frame of reference.
            double nY = -(y_pixels * 2 - 1);
            double nZ = -(z_pixels * 2 - 1);

            double y = VPW / 2 * nY;
            double z = VPH / 2 * nZ;

            TargetInfo target = new TargetInfo(y, z);
            target.setSkew(slope);
            targets.add(target);
        }

        return targets;
    }

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    public static List<double[]> extractTopCornersFromBoundingBoxes(double[] xCorners, double[] yCorners) {
        List<Translation2d> corners = new ArrayList<>();
        for (int i = 0; i < xCorners.length; i++) {
            corners.add(new Translation2d(xCorners[i], yCorners[i]));
        }

        corners.sort(xSort);

        List<Translation2d> left = corners.subList(0, corners.size() / 2);
        List<Translation2d> right = corners.subList(corners.size() / 2, corners.size());

        left.sort(ySort);
        right.sort(ySort);

        List<Translation2d> leftTop = left.subList(0, (corners.size() / 2) / 2);
        List<Translation2d> rightTop = right.subList(0, (corners.size() / 2) / 2);

        leftTop.sort(xSort);
        rightTop.sort(xSort);

        Translation2d leftCorner = leftTop.get(0);
        Translation2d rightCorner = rightTop.get(rightTop.size() - 1);
        return Arrays.asList(new double[]{leftCorner.x(), leftCorner.y()}, new double[]{rightCorner.x(), rightCorner.y()});
    }

    public static List<double[]> extractBottomCornersFromBoundingBoxes(double[] xCorners, double[] yCorners) {
        List<Translation2d> corners = new ArrayList<>();
        for (int i = 0; i < xCorners.length; i++) {
            corners.add(new Translation2d(xCorners[i], yCorners[i]));
        }

        corners.sort(xSort);

        List<Translation2d> left = corners.subList(0, corners.size() / 2);
        List<Translation2d> right = corners.subList(corners.size() / 2, corners.size());

        left.sort(ySort);
        right.sort(ySort);

        Translation2d leftCorner = left.get(left.size() - 1);
        Translation2d rightCorner = right.get(right.size() - 1);

        return Arrays.asList(new double[]{leftCorner.x(), leftCorner.y()}, new double[]{rightCorner.x(), rightCorner.y()});
    }

    public Pose2d getShooterToLens() {
        return mConfig.getShooterToLens();
    }

    public double getLensHeight() {
        return mConfig.getHeight();
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return mConfig.getHorizontalPlaneToLens();
    }

    public void setPipelineConfig(PipelineConfiguration pipelineConfig) {
        mPipelineConfig = pipelineConfig;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + Constants.kImageCaptureLatency;
        mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mPeriodicIO.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);
        mPeriodicIO.xOffset = mNetworkTable.getEntry("tx").getDouble(0.0);
        mPeriodicIO.yOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
        mPeriodicIO.area = mNetworkTable.getEntry("ta").getDouble(0.0);
        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.givenLedMode != mPeriodicIO.ledMode ||
                mPeriodicIO.givenPipeline != mPeriodicIO.pipeline) {
            System.out.println("Table has changed from expected, retrigger!!");
            mOutputsHaveChanged = true;
        }
        if (mOutputsHaveChanged) {
            mNetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(mPeriodicIO.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(mPeriodicIO.pipeline);
            mNetworkTable.getEntry("stream").setNumber(mPeriodicIO.stream);
            mNetworkTable.getEntry("snapshot").setNumber(mPeriodicIO.snapshot);

            mOutputsHaveChanged = false;
        }
    }

    @Override
    public void onStart(Loop.Phase phase) {
        synchronized (Limelight.this) {
            setLed(Limelight.LedMode.OFF);
            RobotState.getInstance(sClassName).resetVision();
            switch (phase) {
                case DISABLED:
                    mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                    break;
                default:
                    mPeriodicIO.schedDeltaDesired = mDefaultSchedDelta;
                    break;
            }
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (Limelight.this) {
            RobotState.getInstance(sClassName).addVisionUpdate(
                    timestamp - getLatency(),
                    getTarget(),
                    Limelight.this);
        }
    }

    @Override
    public void stop() {
        setLed(Limelight.LedMode.OFF);
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean(mConfig.getName() + ": Has Target", mSeesTarget);
        SmartDashboard.putNumber(mConfig.getName() + ": Pipeline Latency (ms)", mPeriodicIO.latency);
        SmartDashboard.putNumber(mConfig.getName() + ": LED Mode", mPeriodicIO.ledMode);
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != mPeriodicIO.ledMode) {
            mPeriodicIO.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void setPipelineNumber(int mode) {
        if (mode != mPeriodicIO.pipeline) {
            RobotState.getInstance(sClassName).resetVision();
            mPeriodicIO.pipeline = mode;

            System.out.println(mPeriodicIO.pipeline + ", " + mode);
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void triggerOutputs() {
        mOutputsHaveChanged = true;
    }

    public synchronized int getPipeline() {
        return mPeriodicIO.pipeline;
    }

    public synchronized List<TargetInfo> getTarget() {
        List<TargetInfo> targets = getRawTargetInfos();

        if (mSeesTarget && targets != null) {
            return targets;
        }

        return null;
    }

    public synchronized List<TargetInfo> getRawTargetInfos() {
        return getRawTargetInfos(
                Constants.kUseTopCorners ? getTopCorners() : getBottomCorners(),
                mPipelineConfig, mTargets, mConfig.getHorizontalFOV(), mConfig.getVerticalFOV());
    }

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    private List<double[]> getTopCorners() {
        double[] corners = mNetworkTable.getEntry("tcornxy").getDoubleArray(mZeroArray);
        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

        // something went wrong
        if (!mSeesTarget || corners.length < 8 || corners == mZeroArray || corners.length % 2 != 0) {
            return null;
        }

        double[] xCorners = new double[corners.length / 2];
        double[] yCorners = new double[corners.length / 2];

        for (int i = 0; i < corners.length; i++) {
            if (i % 2 == 0) {
                xCorners[i / 2] = corners[i];
            } else {
                yCorners[i / 2] = corners[i];
            }
        }

        return extractTopCornersFromBoundingBoxes(xCorners, yCorners);
    }

    private List<double[]> getBottomCorners() {
        double[] corners = mNetworkTable.getEntry("tcornxy").getDoubleArray(mZeroArray);
        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

        // something went wrong
        if (!mSeesTarget || corners.length != 8 || corners == mZeroArray) {
            return null;
        }

        double[] xCorners = new double[corners.length / 2];
        double[] yCorners = new double[corners.length / 2];

        for (int i = 0; i < corners.length; i++) {
            if (i % 2 == 0) {
                xCorners[i / 2] = corners[i];
            } else {
                yCorners[i / 2] = corners[i];
            }
        }

        return extractBottomCornersFromBoundingBoxes(xCorners, yCorners);
    }

    public double getLatency() {
        return mPeriodicIO.latency;
    }

    public LimelightConfig getConstants() {
        return mConfig;
    }

    public void setConstants(LimelightConfig mConstants) {
        this.mConfig = mConstants;
    }

    @Override
    public String getLogHeaders() {
        if (mLoggingEnabled) {
            return mConfig.getName() + ".systemState," +
                    mConfig.getName() + ".latency," +
                    mConfig.getName() + ".givenLedMode," +
                    mConfig.getName() + ".ledMode," +
                    mConfig.getName() + ".givenPipeline," +
                    mConfig.getName() + ".pipeline," +
                    mConfig.getName() + ".xOffset," +
                    mConfig.getName() + ".yOffset," +
                    mConfig.getName() + ".area," +
                    mConfig.getName() + ".tv," +
                    mConfig.getName() + ".rawCorners," +
                    mConfig.getName() + ".rawContours," +
                    mConfig.getName() + ".schedDeltaDesired," +
                    mConfig.getName() + ".schedDeltaActual," +
                    mConfig.getName() + ".schedDuration";
        }
        return null;
    }

    @Override
    public String getLogValues(boolean telemetry) {
        String values;
        if (telemetry) {
            values = mPeriodicIO.latency + "," +
                    mPeriodicIO.givenLedMode + "," +
                    mPeriodicIO.ledMode + "," +
                    mPeriodicIO.givenPipeline + "," +
                    mPeriodicIO.pipeline + "," +
                    mPeriodicIO.xOffset + "," +
                    mPeriodicIO.yOffset + "," +
                    mPeriodicIO.area + ",";
        } else {
            mPeriodicIO.schedDuration = Timer.getFPGATimestamp() - mPeriodicIO.lastSchedStart;

            values = mPeriodicIO.latency + "," +
                    mPeriodicIO.givenLedMode + "," +
                    mPeriodicIO.ledMode + "," +
                    mPeriodicIO.givenPipeline + "," +
                    mPeriodicIO.pipeline + "," +
                    mPeriodicIO.xOffset + "," +
                    mPeriodicIO.yOffset + "," +
                    mPeriodicIO.area + "," +
                    /* mPeriodicIO.rawCorners+ */"\",\"" +
                    /* mPeriodicIO.rawContours+ */"\"," +
                    mPeriodicIO.schedDeltaDesired + "," +
                    mPeriodicIO.schedDeltaActual + "," +
                    mPeriodicIO.schedDuration;
        }
        return values;
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public static class PeriodicIO {
        // LOGGING
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double schedDuration;
        // INPUTS
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;
        // OUTPUTS
        public int ledMode = 1; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
        private double lastSchedStart;
    }
}
