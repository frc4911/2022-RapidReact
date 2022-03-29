package libraries.cheesylib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.Timer;
import libraries.cyberlib.control.FramePeriodSwitch;

/**
 * Creates TalonFX objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class TalonFXFactory {
    private final static int kTimeoutMs = 120;

    public static class Configuration {
        public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        // factory default
        public double NEUTRAL_DEADBAND = 0.04;

//        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_SOFT_LIMIT = true;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public double FORWARD_SOFT_LIMIT = 0;
        public double REVERSE_SOFT_LIMIT = 0;

        public boolean INVERTED = false;
        public boolean SENSOR_PHASE = false;

        public int CONTROL_FRAME_PERIOD_MS = 20;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

        public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;
    }

    private static final TalonFXFactory.Configuration kDefaultConfiguration = new TalonFXFactory.Configuration();
    private static final TalonFXFactory.Configuration kSlaveConfiguration = new TalonFXFactory.Configuration();

    static {
        // This control frame value seems to need to be something reasonable to avoid the Talon's
        // LEDs behaving erratically.  Potentially try to increase as much as possible.
        kSlaveConfiguration.CONTROL_FRAME_PERIOD_MS = 100;
        kSlaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.ENABLE_SOFT_LIMIT = false;
    }

    // create a TalonFX with the default (out of the box) configuration
    public static TalonFX createDefaultTalon(int id) {
        return createTalon(id, kDefaultConfiguration, "");
    }

    public static TalonFX createDefaultTalon(int id, String canivore) {
        return createTalon(id, kDefaultConfiguration, canivore);
    }

    public static TalonFX createPermanentSlaveTalon(int id, int master_id) {
        final TalonFX talon = createTalon(id, kSlaveConfiguration, "");
        talon.set(ControlMode.Follower, master_id);
        return talon;
    }

    public static TalonFX createTalon(int id, TalonFXFactory.Configuration config, String canivore) {
        TalonFX talon;
        if (canivore.length() > 0){
            talon = new LazyTalonFX(id, canivore);
        }
        else {
            talon = new LazyTalonFX(id);
        }
        
        // moving motor config to Subsystems
        // if (FramePeriodSwitch.getFirmwareVersion(talon) < 0){
        //     System.out.println("Failed to get motor firmware version!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        // }

        // FramePeriodSwitch.configFactoryDefaultPermanent(talon);

        // talon.set(ControlMode.PercentOutput, 0.0);

        // talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        // talon.clearMotionProfileHasUnderrun(kTimeoutMs);
        // talon.clearMotionProfileTrajectories();

        // talon.clearStickyFaults(kTimeoutMs);

        // talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        //         LimitSwitchNormal.Disabled, kTimeoutMs);
        // talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        //         LimitSwitchNormal.Disabled, kTimeoutMs);
        // talon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

        // Turn off re-zeroing by default.
        // talon.configSetParameter(
        //         ParamEnum.eClearPositionOnLimitF, 0, 0, 0, kTimeoutMs);
        // talon.configSetParameter(
        //         ParamEnum.eClearPositionOnLimitR, 0, 0, 0, kTimeoutMs);

        // talon.configNominalOutputForward(0, kTimeoutMs);
        // talon.configNominalOutputReverse(0, kTimeoutMs);
        // talon.configNeutralDeadband(config.NEUTRAL_DEADBAND, kTimeoutMs);

        // talon.configPeakOutputForward(1.0, kTimeoutMs);
        // talon.configPeakOutputReverse(-1.0, kTimeoutMs);

        // talon.setNeutralMode(config.NEUTRAL_MODE);

        // talon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, kTimeoutMs);
        // talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, kTimeoutMs);

        // talon.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, kTimeoutMs);
        // talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, kTimeoutMs);
        // talon.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

        // talon.setInverted(config.INVERTED);
        // talon.setSensorPhase(config.SENSOR_PHASE);

        // talon.selectProfileSlot(0, 0);

        // talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, kTimeoutMs);
        // talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, kTimeoutMs);

        // talon.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, kTimeoutMs);
        // talon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, kTimeoutMs);

        // talon.configVoltageCompSaturation(0.0, kTimeoutMs);
        // talon.configVoltageMeasurementFilter(32, kTimeoutMs);
        // talon.enableVoltageCompensation(false);

        // TODO: Consider configuring supply and stator current limits
//        talon.enableCurrentLimit(config.ENABLE_CURRENT_LIMIT);
        // int defaultRefreshRate = 255;
        // int kTimeoutMs2 = 50; // recommended by CTRE tech support
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,defaultRefreshRate,kTimeoutMs2);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,defaultRefreshRate,kTimeoutMs2);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,defaultRefreshRate,kTimeoutMs2);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,defaultRefreshRate,kTimeoutMs2);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc,defaultRefreshRate,kTimeoutMs2);
        // // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus,defaultRefreshRate,kTimeoutMs2);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,defaultRefreshRate,kTimeoutMs2);
        // // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer,defaultRefreshRate,kTimeoutMs2);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current,defaultRefreshRate,kTimeoutMs2);
        // // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,defaultRefreshRate,kTimeoutMs2);
        // // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets,defaultRefreshRate,kTimeoutMs2);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer,defaultRefreshRate,kTimeoutMs2);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1,defaultRefreshRate,kTimeoutMs2);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,defaultRefreshRate,kTimeoutMs2);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1,defaultRefreshRate,kTimeoutMs2);
        // // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus,defaultRefreshRate,kTimeoutMs2);
        // talon.setStatusFramePeriod(StatusFrame.Status_17_Targets1,defaultRefreshRate,kTimeoutMs2);

        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,
        //         config.GENERAL_STATUS_FRAME_RATE_MS, kTimeoutMs);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
        //         config.FEEDBACK_STATUS_FRAME_RATE_MS, kTimeoutMs);

        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,
        //         config.QUAD_ENCODER_STATUS_FRAME_RATE_MS, kTimeoutMs);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
        //         config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, kTimeoutMs);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
        //         config.PULSE_WIDTH_STATUS_FRAME_RATE_MS, kTimeoutMs);

        // brian may want to remove this one
        // talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

        return talon;
    }
}
