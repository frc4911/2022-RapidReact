package libraries.cyberlib.control;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;

public class FramePeriodSwitch {
    TalonFX mFXMotor;

    public FramePeriodSwitch(TalonFX FXMotor){
        this.mFXMotor = FXMotor;
        setFramePeriods();
    }

    void setOneStatusFramePeriod(StatusFrameEnhanced statusFrame, int framePeriod){
        int retries = 5;
        ErrorCode retVal;

        do {
            retVal = mFXMotor.setStatusFramePeriod(statusFrame, framePeriod, 100);
        } while(!retVal.equals(ErrorCode.OK) && retries-- > 0);

        if (!retVal.equals(ErrorCode.OK)){
            System.out.println("setStatusFramePeriod("+statusFrame.toString()+") failed with status "+retVal.toString());
        }
    }

    void setOneControlFramePeriod(ControlFrame controlFrame, int framePeriod){
        final int totalRetries = 5;
        int retries = totalRetries;
        ErrorCode retVal;

        do {
            if (retries != totalRetries){
                Timer.delay(.001);
            }

            retVal = mFXMotor.setControlFramePeriod(controlFrame, framePeriod);
        } while(!retVal.equals(ErrorCode.OK) && retries-->0);

        if (!retVal.equals(ErrorCode.OK)){
            System.out.println("setControlFramePeriod("+controlFrame.toString()+") failed with status "+retVal.toString());
        }
    }

    // It takes 2 to 30 msec to change each control and status frame period so leaving all at the default value (i.e. not changing) 
    // unless there is a ctr error on console or we think it is needed for our usage
    // public void switchToActive(){

    //     if (!mLastRunWasActive){
    //         // setFramePeriods(mActiveFramePeriod);
    //     }
    //     mLastRunWasActive = true;
    // }

    // public void switchToDormant(){
    //     if (mLastRunWasActive){
    //         // setFramePeriods(mDormantFramePeriod);
    //     }
    //     mLastRunWasActive = false;
    // }

    private void setFramePeriods(){
        double start = Timer.getFPGATimestamp();
        // complete list

        // General Control frame for motor control
        // must be kept low (20 msec?) for proper motor function
        // set above
        // mFXMotor.setControlFramePeriod(ControlFrame.Control_3_General, Math.min(20,mActiveFramePeriod));
        // Advanced Control frame for motor control
        // no documentation found, will keep low
        // do {
        //     retVal = mFXMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, Math.min(20,mActiveFramePeriod));
        //     System.out.println("setControlFramePeriod(ControlFrame.Control_4_Advanced) completed with "+retVal.toString());
        //     Timer.delay(.001);
        // } while (retVal != ErrorCode.OK);
        
        // General Control frame for motor control
        // must be kept low (20 msec?) for proper motor function
        setOneControlFramePeriod(ControlFrame.Control_3_General, 19);

        // Advanced Control frame for motor control
        // no documentation found, appears to be unsupported
        // mFXMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, Math.min(20,mActiveFramePeriod));
        
        // Control frame for adding trajectory points from Stream object
        // no documentation found
        // feature not used
        // mFXMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, framePeriod);
        
        // General Status - Applied Motor Output, fault Information, Limit Switch Information
        // can be larger (longer period) for Followers
        // default is 10 msec
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, kTimeout);

        // Feedback for selected sensor on primary PID[0]
        // Selected Sensor Position (PID 0), Selected Sensor Velocity (PID 0), 
        // Brushed Supply Current Measurement, Sticky Fault Information
        // can be larger (longer period) for Followers
        // default is 20
        setOneStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 18);
        
        //Quadrature sensor
        // default > 100 msec
        // not used
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, framePeriod, kTimeout);

        // Analog sensor, motor controller temperature, and voltage at input leads
        // default > 100 msec
        // temp might be of interest otherwise not used
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, framePeriod, kTimeout);

        // Miscellaneous signals
        // no documentation
        // setOneStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, framePeriod, 100);

        // Communication status
        // no documentation
        // setOneStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, framePeriod, 100);

        // Pulse width sensor
        // default > 100 msec
        // not used
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, framePeriod, kTimeout);

        // Motion profile buffer status
        // not used
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, framePeriod, kTimeout);

        // Brushless Current Status Includes Stator and Supply Current for Talon FX
        // default 50 msec
        // reading in outputTelemetry so anything under 250 should be ok
        // setOneStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 18);
        
        // Motion Profiling/Motion Magic Information
        // default > 100 msec
        // setting for all even though not universally used
        setOneStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 20);

        // Gadgeteer status
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, framePeriod, kTimeout);
        
        // Selected Sensor Position (Aux PID 1)
        // Selected Sensor Velocity (Aux PID 1)
        // default > 100 msec
        // not used
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, framePeriod, kTimeout);

        // Primary PID
        // setting for all even though not universally used
        setOneStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20);

        // Auxiliary PID
        // not used
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, framePeriod, kTimeout);

        // Firmware & API
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus,framePeriod,kTimeout);

        // MotionProfile Targets for Auxiliary PID1
        // not used
        // mFXMotor.setStatusFramePeriod(StatusFrame.Status_17_Targets1,framePeriod,kTimeout);

        System.out.println("setFramePeriods took "+(Timer.getFPGATimestamp()-start));
    }
}