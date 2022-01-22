package frc.robot.subsystems.LimeLights;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.Target;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.vision.TargetInfo;

import java.util.ArrayList;
import java.util.List;

/**
 * Subsystem for interacting with the Limelight 2 or 2+
 */
public abstract class LimeLight extends Subsystem {

    public static class LimelightConstants {
        public String kName = "";
        public String kTableName = "";
        public double kHeight = 0.0;
        public Pose2d kSubsystemToLens = Pose2d.identity();
        public Rotation2d kHorizontalPlaneToLens = Rotation2d.identity();
        // assign a zoom to each pipeline in use (EVERY pipleine must have a zoom!!!)
        public int[] kPipelineZoom = new int[] {1}; // all pipeline zooms default to 1
        public double[] kExpectedTargetCount = new double[] {0, 0};
        public Target[] kTargets = new Target[0];
    }

    protected final double[] mZeroArray = new double[] { 0, 0, 0, 0 };
    protected final double mZeroDouble = 0.0;

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    protected enum TargetType {
        MAIN_CONTOUR, RAW_CORNERS, RAW_CONTOURS
    }

    public enum SystemState {
        HOLDING,
        TARGETING,
        TARGETINGLED
    }

    public enum WantedState {
        HOLD,
        TARGET,
        TARGETLED
    }

    private SystemState mSystemState = SystemState.HOLDING;
    private WantedState mWantedState = WantedState.HOLD;
    private PeriodicIO mPeriodicIO;
    private boolean mStateChanged;

    protected NetworkTable mNetworkTable;
    protected LimelightConstants mConstants = null;
    protected boolean mOutputsHaveChanged = true;

    protected boolean mUsingRawCorners = false;
    protected boolean mUsingRawContours = false;
    protected boolean mUsingSnapShots = false;

    protected ArrayList<TargetInfo> mTargets = new ArrayList<>();

    private ArrayList<Translation2d> mCachedNormCoordinates;
    private int mCachedZoom;

    private final boolean mLoggingEnabled = true;
    @SuppressWarnings("unused")
    private double mLastStart;
    private int mDefaultSchedDelta = 20;
    @SuppressWarnings("unused")
    private int mListIndex;
    private RobotState mRobotState;

    public LimeLight(LimelightConstants constants) {
        mConstants = constants; // set constants
        mPeriodicIO = new PeriodicIO();
        mCachedNormCoordinates = new ArrayList<Translation2d>();
        mCachedZoom = getZoom();
        mNetworkTable = NetworkTableInstance.getDefault().getTable(constants.kTableName);
        // check if network table is found
        if (!hasTable()) {
            System.out.println("No Network table for " + mConstants.kName + ". Check constants.");
        }
        mRobotState = RobotState.getInstance("Limelight");
    }

    public boolean hasTable() {
        return mNetworkTable.getEntry("tv").getDouble(-1.0) != -1.0;
    }

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(Phase phase) {
            synchronized(LimeLight.this) {
                mSystemState = SystemState.HOLDING;
                mWantedState = WantedState.HOLD;
                mStateChanged = true;
                System.out.println(mConstants.kName + " state " + mSystemState);
                switch (phase) {
                    case DISABLED:
                        mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                        break;
                    default:
                        mPeriodicIO.schedDeltaDesired = mDefaultSchedDelta;
                        break;
                }
                stop();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(LimeLight.this) {
                SystemState newState;
                switch(mSystemState) {
                    case TARGETING:
                        newState = handleTargeting();
                        break;
                    case TARGETINGLED:
                        newState = handleTargetingLED();
                        break;
                    case HOLDING:
                    default:
                        newState = handleHolding();
                }

                if (newState != mSystemState) {
                    System.out.println(mConstants.kName + " state " + mSystemState + " (" + timestamp + ")");
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case TARGET:
                return SystemState.TARGETING;
            case TARGETLED:
                return SystemState.TARGETINGLED;
            case HOLD:
            default:
                return SystemState.HOLDING;
        }
    }

    public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }

    private SystemState handleHolding() {
        if (mStateChanged) {
            setLed(LedMode.OFF);
        }

        return defaultStateTransfer();
    }


    private SystemState handleTargeting() {
        if (mStateChanged) {
            setLed(LedMode.OFF);
            setSnapshot();
        }
        addVisionUpdate();

        return defaultStateTransfer();
    }

    private SystemState handleTargetingLED() {
        if (mStateChanged) {
            setLed(LedMode.ON);
            setSnapshot();
        }
        addVisionUpdate();

        return defaultStateTransfer();
    }

    public synchronized void addVisionUpdate() {
        mTargets.clear();
        getRawTargetInfos(mTargets);
        mRobotState.addVisionUpdate(
                Timer.getFPGATimestamp() - getLatency(),
                mTargets,
                mConstants
        );
    }

    /**
     * adds targets to the target list using addTargets()
     */
    protected abstract void getRawTargetInfos(ArrayList<TargetInfo> targets);

    @Override
    public void stop() {
        zeroSensors();
    }

    @Override
    public void zeroSensors() {
        mPeriodicIO.snapshot = 0;
        setPipeline(0);
        setLed(LedMode.OFF);
        mOutputsHaveChanged = true;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mListIndex = enabledLooper.register(mLoop);
    }


    public Pose2d getSubsystemToLens() {
        return mConstants.kSubsystemToLens;
    }

    public double getLensHeight() {
        return mConstants.kHeight;
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return mConstants.kHorizontalPlaneToLens;
    }

    public double[] getExpectedTargetCount() {
        return mConstants.kExpectedTargetCount;
    }

    public boolean getStateChanged(){
        return mStateChanged;
    }

    public synchronized double getLatency() {
        return mPeriodicIO.latency;
    }

    public synchronized int getPipeline() {
        return mPeriodicIO.pipeline;
    }

    // pipline determines zoom
    public synchronized int getZoom() {
        return mConstants.kPipelineZoom[mPeriodicIO.givenPipeline];
    }

    public synchronized int getCachedZoom() {
        return mCachedZoom;
    }

    public synchronized boolean seesTarget() {
        return mNetworkTable.getEntry("tv").getDouble(mZeroDouble) == 1.0;
    }

    public synchronized void useRawCorners(boolean useCorners) {
        mUsingRawCorners = useCorners;
    }

    public synchronized void useRawContours(boolean useContours) {
        mUsingRawContours = useContours;
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != mPeriodicIO.ledMode) {
            mPeriodicIO.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    /**
     * Switches limelight pipelines and automatically accounts for zoom
     */
    public synchronized boolean setPipeline(int mode) {
        if (mode != mPeriodicIO.pipeline) {
            if(mode >= 0 && mode < mConstants.kPipelineZoom.length) {
                mPeriodicIO.pipeline = mode;
                System.out.println(mPeriodicIO.pipeline + ", " + mode);
                mOutputsHaveChanged = true;
            } else {
                System.out.println("Failed to switch pipelines. Pipline #" + mode + " does not exist for " + mConstants.kName + ".");
                return false;
            }
        }
        return true;
    }

    public synchronized void useSnapShots(boolean useSnapShots) {
        mUsingSnapShots = useSnapShots;
    }

    private synchronized void setSnapshot() {
        if (mUsingSnapShots) {
            mPeriodicIO.snapshot = 1;
            mOutputsHaveChanged = true;
        }

    }

    public synchronized void triggerOutputs() {
        mOutputsHaveChanged = true;
    }

    /**
     * Gets all the data types as normalized coordinates. always use addTargets(), before calling this a 2nd time.
     * (make sure data type is being used ie. useRawCorners(true) and limelight is sending the given DataType)
     *
     * @return the requested data type in angle offset units (like xOffset) else empty ArrayList
     */
    protected synchronized ArrayList<Translation2d> get(TargetType dataType) {
        mCachedNormCoordinates.clear();
        mCachedZoom = getZoom();
        switch (dataType) {
            case MAIN_CONTOUR:
                storeMainContourToNormCoord();
                return mCachedNormCoordinates;
            case RAW_CORNERS:
                storeRawCornersToNormCoord();
                return mCachedNormCoordinates;
            case RAW_CONTOURS:
                storeRawContoursToNormCoord();
                return mCachedNormCoordinates;
        }
        return mCachedNormCoordinates;
    }

    private void storeMainContourToNormCoord() {
        mCachedNormCoordinates.add(new Translation2d(
                (2 * mCachedZoom * Math.tan(Math.toRadians(mPeriodicIO.xOffset)) / Constants.kVPW ),
                (2 * mCachedZoom * Math.tan(Math.toRadians(mPeriodicIO.yOffset)) / Constants.kVPH )
        ));
    }

    private void storeRawCornersToNormCoord() {
        for (Translation2d corners : mPeriodicIO.rawCorners) {
            mCachedNormCoordinates.add(new Translation2d(
                    -(corners.x() - 160) / 160,
                    -(corners.y() - 120) / 120
            ));
        }
    }

    private void storeRawContoursToNormCoord() {
        for(int i = 0; i < mPeriodicIO.rawContours.size(); i++) {
//            System.out.println("Raw contour #" + i + ": " + mPeriodicIO.rawContours.get(i).toString());
        }
        for (Translation2d contours : mPeriodicIO.rawContours) {
            mCachedNormCoordinates.add(new Translation2d(
                    contours.x(),
                    contours.y()
            ));
        }
    }

    /**
     * use this method to add targets to the target list
     *
     * @param targets in angle normalized coordinate units (as given by get())
     */
    public synchronized ArrayList<TargetInfo> add(ArrayList<TargetInfo> targets, ArrayList<Translation2d> rawTargets) {
        for(Translation2d coordinate : rawTargets) {
            targets.add(new TargetInfo(
                    coordinate.x() * Constants.kVPW / (2 * mCachedZoom),
                    coordinate.y() * Constants.kVPH / (2 * mCachedZoom)
            ));
        }
        return targets;
    }

    public synchronized void add(ArrayList<TargetInfo> targets, TargetType targetType) {
        add(targets, get(targetType));
    }

    /**
     * Used for zooming, this method checks if all cached points are in a rectangle
     * @param proportionOfFOV Scales down the rectangle and must be less than or equal to 1. (if 1, then rectangle approximates entire FOV)
     * @return whether the given points are in the rectangle approximate
     */
    protected synchronized boolean cachedPointsInRect(double proportionOfFOV) {
        boolean inRect = !mCachedNormCoordinates.isEmpty(); // there must be corners for them to be in the rectApprox
        for(Translation2d coordinate : mCachedNormCoordinates) {
            inRect = inRect && (pointInRect(coordinate, proportionOfFOV));
        }
        return inRect;
    }

    // rearranging the equation lim(a => infinity) (r^2a > x^2a + y^2a),
    // gives: 1 > lim(a => infinity) (x^2a + y^2a)/r^2a = lim(a => infinity) (x / r)^2a + (y / r)^2a
    // the right side of the equation equals 0 if (x/r)^2 < 1 and (y/r)^2 < 1, else the equation diverges
    private boolean pointInRect(Translation2d coordinate, double proportionOfFOV) {
        return (Math.pow((coordinate.x()/proportionOfFOV), 2) < 1) && (Math.pow((coordinate.y()/proportionOfFOV), 2) < 1);
    }

    private List<Translation2d> getCoordinates(double[] rawCorners) {
        List<Translation2d> coordinates = new ArrayList<>();

        if (rawCorners == null || rawCorners.length % 2 == 1) {
            return coordinates;
        }

        for (int i = 0; i < rawCorners.length; i += 2) {
            coordinates.add(new Translation2d(rawCorners[i], rawCorners[i + 1]));
        }

        return coordinates;
    }

    private List<Translation2d> getCoordinates(double[] x, double[] y) {
        List<Translation2d> coordinates = new ArrayList<>();
        if ((x.length != y.length) || x.equals(mZeroArray) || y.equals(mZeroArray)) {
            return coordinates;
        }

        for (int i = 0; i < x.length; i++) {
            coordinates.add(new Translation2d(x[i], y[i]));
        }

        return coordinates;
    }

    private List<Translation2d> getCoordinates(double x0, double y0, double x1, double y1, double x2, double y2) {
        List<Translation2d> coordinates = new ArrayList<Translation2d>();
        if (x0 != mZeroDouble && y0 != mZeroDouble) {
            coordinates.add(new Translation2d(x0, y0));
        }

        if (x1 != mZeroDouble && y1 != mZeroDouble) {
            coordinates.add(new Translation2d(x1, y1));
        }

        if (x2 != mZeroDouble && y2 != mZeroDouble) {
            coordinates.add(new Translation2d(x2, y2));
        }
        return coordinates;
    }

    @Override
    public String getLogHeaders() {
        if (mLoggingEnabled){
            return  mConstants.kName+".systemState,"+
                    mConstants.kName+".latency,"+
                    mConstants.kName+".givenLedMode,"+
                    mConstants.kName+".ledMode,"+
                    mConstants.kName+".givenPipeline,"+
                    mConstants.kName+".pipeline,"+
                    mConstants.kName+".xOffset,"+
                    mConstants.kName+".yOffset,"+
                    mConstants.kName+".area,"+
                    mConstants.kName+".tv,"+
                    mConstants.kName+".rawCorners,"+
                    mConstants.kName+".rawContours,"+
                    mConstants.kName+".schedDeltaDesired,"+
                    mConstants.kName+".schedDeltaActual,"+
                    mConstants.kName+".schedDuration";
        }
        return null;
    }

    private String generateLogValues(boolean telemetry){
        String values;
        if (telemetry) {
            values = ""+mSystemState+","+
                    mPeriodicIO.latency+","+
                    mPeriodicIO.givenLedMode+","+
                    mPeriodicIO.ledMode+","+
                    mPeriodicIO.givenPipeline+","+
                    mPeriodicIO.pipeline+","+
                    mPeriodicIO.xOffset+","+
                    mPeriodicIO.yOffset+","+
                    mPeriodicIO.area+","+
                    mPeriodicIO.tv+",\""+
                    /*mPeriodicIO.rawCorners+*/"\",\""+
                    /*mPeriodicIO.rawContours+*/"\","+
                    /*mPeriodicIO.schedDeltaDesired+*/","+
                    /*mPeriodicIO.schedDeltaActual+*/","
            /*mPeriodicIO.schedDuration*/;
        }
        else {
            mPeriodicIO.schedDuration = Timer.getFPGATimestamp() - mPeriodicIO.lastSchedStart;

            values = ""+mSystemState+","+
                    mPeriodicIO.latency+","+
                    mPeriodicIO.givenLedMode+","+
                    mPeriodicIO.ledMode+","+
                    mPeriodicIO.givenPipeline+","+
                    mPeriodicIO.pipeline+","+
                    mPeriodicIO.xOffset+","+
                    mPeriodicIO.yOffset+","+
                    mPeriodicIO.area+","+
                    mPeriodicIO.tv+",\""+
                    /*mPeriodicIO.rawCorners+*/"\",\""+
                    /*mPeriodicIO.rawContours+*/"\","+
                    mPeriodicIO.schedDeltaDesired+","+
                    mPeriodicIO.schedDeltaActual+","+
                    mPeriodicIO.schedDuration;
        }
        return values;
    }

    @Override
    public String getLogValues(boolean telemetry) {
        if (mLoggingEnabled){
            return generateLogValues(telemetry);
        }
        return null;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        double now                   = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart   = now;

        mPeriodicIO.currentState = mSystemState;
        mPeriodicIO.latency = mNetworkTable.getEntry("tl").getDouble(mZeroDouble) / 1000.0
                + Constants.kImageCaptureLatency;
        mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mPeriodicIO.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(mZeroDouble);
        mPeriodicIO.xOffset = mNetworkTable.getEntry("tx").getDouble(mZeroDouble);
        mPeriodicIO.yOffset = mNetworkTable.getEntry("ty").getDouble(mZeroDouble);
        mPeriodicIO.area = mNetworkTable.getEntry("ta").getDouble(mZeroDouble);
        mPeriodicIO.tv = mNetworkTable.getEntry("tv").getDouble(-1.0);

        mPeriodicIO.rawCorners.clear();
        if (mUsingRawCorners) {
            // if network table uses tcornxy entry
            double[] rawXYCorners = mNetworkTable.getEntry("tcornxy").getDoubleArray(mZeroArray);
            if (!rawXYCorners.equals(mZeroArray)) {
                mPeriodicIO.rawCorners.addAll(getCoordinates(rawXYCorners));

                // if network table uses tcornx and tcorny entry
            } else {
                double[] rawXCorners = mNetworkTable.getEntry("tcornx").getDoubleArray(mZeroArray);
                double[] rawYCorners = mNetworkTable.getEntry("tcorny").getDoubleArray(mZeroArray);
                if (!rawXCorners.equals(mZeroArray) && !rawYCorners.equals(mZeroArray)) {
                    mPeriodicIO.rawCorners.addAll(getCoordinates(rawXCorners, rawYCorners));
                }
            }
        }

        mPeriodicIO.rawContours.clear();
        if (mUsingRawContours) {
            mPeriodicIO.rawContours.addAll(getCoordinates(mNetworkTable.getEntry("tx0").getDouble(mZeroDouble),
                    mNetworkTable.getEntry("ty0").getDouble(mZeroDouble),
                    mNetworkTable.getEntry("tx1").getDouble(mZeroDouble),
                    mNetworkTable.getEntry("ty1").getDouble(mZeroDouble),
                    mNetworkTable.getEntry("tx2").getDouble(mZeroDouble),
                    mNetworkTable.getEntry("ty2").getDouble(mZeroDouble)));
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.givenLedMode != mPeriodicIO.ledMode || mPeriodicIO.givenPipeline != mPeriodicIO.pipeline) {
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

    int delta = 1;
    @Override
    public int whenRunAgain () {
        if (mStateChanged && mPeriodicIO.schedDeltaDesired==0){
            return 1; // one more loop before going to sleep
        }
        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean(mConstants.kName + ": Has Target", seesTarget());
        SmartDashboard.putNumber(mConstants.kName + ": Pipeline Latency (ms)", mPeriodicIO.latency);
    }

    public class PeriodicIO {
        // LOGGING
        public  int    schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;

        // INPUTS
        public SystemState currentState;
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;
        public double tv;
        public List<Translation2d> rawCorners = new ArrayList<>();
        public List<Translation2d> rawContours = new ArrayList<>();

        // OUTPUTS
        public int ledMode = 1; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }
}
