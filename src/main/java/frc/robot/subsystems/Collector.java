package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;
import libraries.cheesylib.util.LatchedBoolean;

public class Collector extends Subsystem {

    // Hardware
    private final TalonFX mFXCollector;
    private final Solenoid mSolenoid;

    // Subsystem Constants
    private final double kCollectSpeed = 0.5;

    // Configuration Constants
    private final double kCurrentLimit = 60;

    // Subsystem States
    public enum SolenoidState {
        EXTEND(true),
        RETRACT(false);

        private final boolean state;

        private SolenoidState(boolean state) {
            this.state = state;
        }

        public boolean get() {
            return state;
        }
    }

    public enum SystemState {
        HOLDING,
        COLLECTING,
        BACKING
    }

    public enum WantedState {
        HOLD,
        COLLECT,
        BACK
    }

    private SystemState mSystemState;
    private WantedState mWantedState;
    private boolean mStateChanged;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private LatchedBoolean mSystemStateChange = new LatchedBoolean();
    private SolenoidState mSolenoidState;
    private boolean mRunCollectorLoop;
    private double lastBackingTimestamp;

    double collectSpeed;

    // Other
    private SubsystemManager mSubsystemManager;

    // Subsystem Creation
    private static String sClassName;
    private static int sInstanceCount;
    private static Collector sInstance = null;

    public static Collector getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new Collector(caller);
        } else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller) {
        System.out.println("(" + caller + ") " + " getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Collector(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mFXCollector = TalonFXFactory.createDefaultTalon(Ports.COLLECTOR);
        mSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.COLLECTOR_DEPLOY);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        collectSpeed = SmartDashboard.getNumber("Collecting Speed", -1.0);
        if (collectSpeed == -1) {
            SmartDashboard.putNumber("Collecting Speed", 0.0);
        }
        configMotors();
    }

    private void configMotors() {
        mFXCollector.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXCollector.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXCollector.setControlFramePeriod(ControlFrame.Control_3_General, 18);

        mFXCollector.setInverted(false);

        mFXCollector.setNeutralMode(NeutralMode.Coast);

        mFXCollector
                .configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, kCurrentLimit, kCurrentLimit, 0));

    }

    @Override
    public void onStart(Phase phase) {
        synchronized (Collector.this) { // TODO Check if the key word synchronized is needed
            mSystemState = SystemState.HOLDING;
            mWantedState = WantedState.HOLD;
            mStateChanged = true;
            mRunCollectorLoop = false;
            System.out.println(sClassName + " state " + mSystemState);
            // this subsystem is "on demand" so
            mPeriodicIO.schedDeltaDesired = 0;
            stop(); // put into a known state
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (Collector.this) {
            do {
                SystemState newState;
                switch (mSystemState) {
                    case COLLECTING:
                        newState = handleCollecting();
                        break;
                    case BACKING:
                        newState = handleBacking();
                        break;
                    case HOLDING:
                    default:
                        newState = handleHolding();
                        break;
                }

                if (newState != mSystemState) {
                    System.out.println(
                            sClassName + " state " + mSystemState + " to " + newState + " (" + timestamp + ")");
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            } while (mSystemStateChange.update(mStateChanged));
        }
    }

    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState) {
            mSubsystemManager.scheduleMe(mListIndex, 1, false);
            System.out.println("waking " + sClassName);
        }

        mWantedState = state;
    }

    private SystemState handleHolding() {
        if (mStateChanged) {
            mPeriodicIO.collectorDemand = 0.0;
            mPeriodicIO.solenoidDemand = SolenoidState.RETRACT;
        }

        return defaultStateTransfer();
    }

    private SystemState handleCollecting() {
        collectSpeed = SmartDashboard.getNumber("Collecting Speed", 0.0);
        updateCollector(collectSpeed);
        // updateCollector(kCollectSpeed);

        return defaultStateTransfer();
    }

    private SystemState handleBacking() {
        if (mStateChanged) {
            lastBackingTimestamp = Timer.getFPGATimestamp();
            mPeriodicIO.schedDeltaDesired = 20;
        }
        double now = Timer.getFPGATimestamp();

        if (now - lastBackingTimestamp < 0.5) {
            updateCollector(kCollectSpeed);
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta;
            mRunCollectorLoop = true;
        } else {
            updateCollector(-kCollectSpeed);
        }

        return defaultStateTransfer();
    }

    private void updateCollector(double speed) {
        // Run one loop after extending so wheels do not run while retracted
        if (mRunCollectorLoop) {
            mPeriodicIO.collectorDemand = speed;
            mPeriodicIO.schedDeltaDesired = 0;
            mRunCollectorLoop = false;
        }
        if (mStateChanged) {
            mPeriodicIO.solenoidDemand = SolenoidState.EXTEND;
            mPeriodicIO.schedDeltaDesired = mPeriodicIO.mDefaultSchedDelta; // run one more time in 100 ms for collector
                                                                            // startup
            mRunCollectorLoop = true;
        }
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case COLLECT:
                return SystemState.COLLECTING;
            case BACK:
                return SystemState.BACKING;
            case HOLD:
            default:
                return SystemState.HOLDING;
        }
    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;
    }

    @Override
    public void writePeriodicOutputs() {
        if (mSolenoidState != mPeriodicIO.solenoidDemand) {
            mSolenoidState = mPeriodicIO.solenoidDemand;
            mSolenoid.set(mPeriodicIO.solenoidDemand.get());
        }
        mFXCollector.set(ControlMode.PercentOutput, mPeriodicIO.collectorDemand);
    }

    @Override
    public void stop() {
        mFXCollector.set(ControlMode.PercentOutput, 0.0);
        mSolenoid.set(SolenoidState.RETRACT.get());

        mPeriodicIO.collectorDemand = 0.0;
        mSolenoidState = SolenoidState.RETRACT;
    }

    @Override
    public int whenRunAgain() {
        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public String getLogHeaders() {
        return "Collector";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        return "Collector.Values";
    }

    @Override
    public void outputTelemetry() {

    }

    public static class PeriodicIO {
        // Logging
        private final int mDefaultSchedDelta = 100; // axis updated every 100 msec
        private int schedDeltaDesired;
        public double schedDeltaActual;
        public double schedDuration;
        private double lastSchedStart;

        // Inputs

        // Outputs
        private double collectorDemand;
        private SolenoidState solenoidDemand;
    }

}