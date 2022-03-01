package libraries.cyberlib.utils;

import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderFaults;

public class CheckFaults {

    public String getFaults(TalonFX motor) {
        String errorFaults = "";
        StickyFaults sf = new StickyFaults();
        motor.getStickyFaults(sf);
        if (sf.hasAnyFault()) {
            if (sf.ForwardLimitSwitch) {
                errorFaults += "ForwardLimitSwitch-";
            }
            if (sf.ForwardSoftLimit) {
                errorFaults += "ForwardSoftLimit-";
            }
            if (sf.RemoteLossOfSignal) {
                errorFaults += "RemoteLossOfSignal-";
            }
            if (sf.HardwareESDReset) {
                errorFaults += "HardwareESDReset-";
            }
            if (sf.ResetDuringEn) {
                errorFaults += "ResetDuringEn-";
            }
            if (sf.ReverseLimitSwitch) {
                errorFaults += "ReverseLimitSwitch-";
            }
            if (sf.ReverseSoftLimit) {
                errorFaults += "ReverseSoftLimit-";
            }
            if (sf.SensorOutOfPhase) {
                errorFaults += "SensorOutOfPhase-";
            }
            if (sf.SensorOverflow) {
                errorFaults += "SensorOverflow-";
            }
            if (sf.UnderVoltage) {
                errorFaults += "UnderVoltage-";
            }
            if (sf.SupplyOverV) {
                errorFaults += "SupplyOverV-";
            }
            if (sf.SupplyUnstable) {
                errorFaults += "SupplyUnstable-";
            }
            if (sf.APIError) {
                errorFaults += "APIError-";
            }
        }

        return errorFaults;
    }

    public void clearFaults(TalonFX motor) {
        motor.clearStickyFaults();
    }

    public String getFaults(TalonSRX motor) {
        String errorFaults = "";
        StickyFaults sf = new StickyFaults();
        motor.getStickyFaults(sf);
        if (sf.hasAnyFault()) {
            if (sf.ForwardLimitSwitch) {
                errorFaults += "ForwardLimitSwitch-";
            }
            if (sf.ForwardSoftLimit) {
                errorFaults += "ForwardSoftLimit-";
            }
            if (sf.RemoteLossOfSignal) {
                errorFaults += "RemoteLossOfSignal-";
            }
            if (sf.HardwareESDReset) {
                errorFaults += "HardwareESDReset-";
            }
            if (sf.ResetDuringEn) {
                errorFaults += "ResetDuringEn-";
            }
            if (sf.ReverseLimitSwitch) {
                errorFaults += "ReverseLimitSwitch-";
            }
            if (sf.ReverseSoftLimit) {
                errorFaults += "ReverseSoftLimit-";
            }
            if (sf.SensorOutOfPhase) {
                errorFaults += "SensorOutOfPhase-";
            }
            if (sf.SensorOverflow) {
                errorFaults += "SensorOverflow-";
            }
            if (sf.UnderVoltage) {
                errorFaults += "UnderVoltage-";
            }
        }

        return errorFaults;
    }

    public void clearFaults(TalonSRX motor) {
        motor.clearStickyFaults();
    }

    // not currently supported - check in future updates
    // public boolean getPigeonFaults(PigeonIMU pigeon) {
    // String errorFaults = "";
    // PigeonIMU_Faults sf = new PigeonIMU_Faults();
    // pigeon.getFaults(sf);
    // if (sf.hasAnyFault()) {
    // errorFaults = "PigeonFault";
    // } else {
    // errorFaults = "none";
    // }
    // SmartDashboard.putString("Pigeon"-errorFaults);
    // return sf.hasAnyFault();
    // }

    // public void clearPigeonFaults(PigeonIMU pigeon) {
    // pigeon.clearStickyFaults();
    // }

    public String getFaults(CANCoder CANCoder) {
        String errorFaults = "";
        CANCoderFaults ccf = new CANCoderFaults();
        CANCoder.getFaults(ccf);
        if (ccf.hasAnyFault()) {
            if (ccf.APIError) {
                errorFaults += "APIError-";
            }
            if (ccf.HardwareFault) {
                errorFaults += "HardwareFault-";
            }
            if (ccf.MagnetTooWeak) {
                errorFaults += "MagnetTooWeak-";
            }
            if (ccf.ResetDuringEn) {
                errorFaults += "ResetDuringEn-";
            }
            if (ccf.UnderVoltage) {
                errorFaults += "UnderVoltage-";
            }
        }
        return errorFaults;
    }

    public void clearFaults(CANCoder CANCoder) {
        CANCoder.clearStickyFaults();
    }
}
