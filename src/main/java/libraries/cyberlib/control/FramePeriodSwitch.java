package libraries.cyberlib.control;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;

public class FramePeriodSwitch {

    private FramePeriodSwitch(){
    }

    @SuppressWarnings("unused")
    private static void setOneStatusFramePeriod(TalonFX FXMotor, StatusFrameEnhanced statusFrame, int framePeriod){
        int retries = 5;
        ErrorCode retVal;

        do {
            retVal = FXMotor.setStatusFramePeriod(statusFrame, framePeriod, 100);
        } while(!retVal.equals(ErrorCode.OK) && retries-- > 0);

        if (!retVal.equals(ErrorCode.OK)){
            System.out.println("setStatusFramePeriod("+statusFrame.toString()+") failed with status "+retVal.toString());
        }
    }

    @SuppressWarnings("unused")
    private static void setOneControlFramePeriod(TalonFX FXMotor, ControlFrame controlFrame, int framePeriod){
        final int totalRetries = 5;
        int retries = totalRetries;
        ErrorCode retVal;

        do {
            if (retries != totalRetries){
                Timer.delay(.001);
            }

            retVal = FXMotor.setControlFramePeriod(controlFrame, framePeriod);
        } while(!retVal.equals(ErrorCode.OK) && retries-->0);

        if (!retVal.equals(ErrorCode.OK)){
            System.out.println("setControlFramePeriod("+controlFrame.toString()+") failed with status "+retVal.toString());
        }
    }

    private enum GetRoutine{
        STATORCURRENT,
        SENSORPOSITION,
        FIRMWAREVERSION
    }

    private static double getRoutine(TalonFX FXMotor, GetRoutine getType){
        final int kRetries =5;
        double retries = kRetries;
        ErrorCode error;
        double retVal = 0;
        double delay = .001;
        double start = Timer.getFPGATimestamp();
        do {
            if (retries != kRetries){
                Timer.delay(delay);
            }
            switch (getType){
                case STATORCURRENT:
                    retVal = FXMotor.getStatorCurrent();
                    break;
                case SENSORPOSITION:
                    retVal = FXMotor.getSelectedSensorPosition();
                    delay = 0;
                    break;
                case FIRMWAREVERSION:
                    retVal = FXMotor.getFirmwareVersion();
                    break;
            }
            error = FXMotor.getLastError();
        } while (!error.equals(ErrorCode.OK) && ((retries--)>0));
        
        if (!error.equals(ErrorCode.OK)){
            System.out.println("Failed "+getType+"(), time needed "+(Timer.getFPGATimestamp()-start));
        }
        return retVal;
    }

    static StatusFrameEnhanced[] framePeriods = {
        StatusFrameEnhanced.Status_1_General, // getStatusFramePeriod returns 10 and 240 with getLastError() == ErrorCode.OK
        StatusFrameEnhanced.Status_2_Feedback0, // getStatusFramePeriod returns 10 with getLastError() == ErrorCode.OK
        StatusFrameEnhanced.Status_3_Quadrature, // getStatusFramePeriod returns 20 with getLastError() == ErrorCode.OK
        StatusFrameEnhanced.Status_4_AinTempVbat, // getStatusFramePeriod returns 240 with getLastError() == ErrorCode.OK
        StatusFrameEnhanced.Status_6_Misc, // getStatusFramePeriod returns 240 with getLastError() == ErrorCode.OK
        StatusFrameEnhanced.Status_7_CommStatus, // getStatusFramePeriod returns 0 with getLastError() == ErrorCode.OK
        StatusFrameEnhanced.Status_8_PulseWidth, // getStatusFramePeriod returns 0 with getLastError() == ErrorCode.OK
        StatusFrameEnhanced.Status_9_MotProfBuffer, // getStatusFramePeriod returns 0 with getLastError() == ErrorCode.OK
        StatusFrameEnhanced.Status_10_Targets, // getStatusFramePeriod returns 0 and 20 with getLastError() == ErrorCode.OK
        StatusFrameEnhanced.Status_11_UartGadgeteer, // getStatusFramePeriod returns 0 and 20 with getLastError() == ErrorCode.OK
        StatusFrameEnhanced.Status_12_Feedback1, // getStatusFramePeriod returns 0 and 240 with getLastError() == ErrorCode.OK
        StatusFrameEnhanced.Status_13_Base_PIDF0, // getStatusFramePeriod returns 240 with getLastError() == ErrorCode.OK
        StatusFrameEnhanced.Status_14_Turn_PIDF1, // getStatusFramePeriod returns 240 with getLastError() == ErrorCode.OK
        StatusFrameEnhanced.Status_15_FirmwareApiStatus, // getStatusFramePeriod returns 240 with getLastError() == ErrorCode.OK
    };
    public static void setFramePeriodsVolatile(TalonFX FXMotor){
        double start = Timer.getFPGATimestamp();

        // System.out.println("last error "+FXMotor.getLastError());
        // System.out.println("last error "+FXMotor.getLastError());
        // System.out.println("last error "+FXMotor.getLastError());

        // ErrorCode error;
        // double period;
        // int retries;
        // for (int i=0; i<framePeriods.length; i++){
        //     retries = 5;
        //     do{
        //         period = FXMotor.getStatusFramePeriod(framePeriods[i]);
        //         error = FXMotor.getLastError();
        //         System.out.println("statusFramePeriod for "+framePeriods[i].toString()+" is "+period +" with "+error.toString());
                
        //         Timer.delay(.050);
        //     } while(/*!error.equals(ErrorCode.OK)*/ (retries-->0) && period==0);
        // }

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
        // setOneControlFramePeriod(FXMotor, ControlFrame.Control_3_General, 19);

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
        // setOneStatusFramePeriod(FXMotor, StatusFrameEnhanced.Status_2_Feedback0, 18);
        
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
        // setOneStatusFramePeriod(FXMotor, StatusFrameEnhanced.Status_10_Targets, 20);

        // Gadgeteer status
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, framePeriod, kTimeout);
        
        // Selected Sensor Position (Aux PID 1)
        // Selected Sensor Velocity (Aux PID 1)
        // default > 100 msec
        // not used
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, framePeriod, kTimeout);

        // Primary PID
        // setting for all even though not universally used
        // setOneStatusFramePeriod(FXMotor, StatusFrameEnhanced.Status_13_Base_PIDF0, 20);

        // Auxiliary PID
        // not used
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, framePeriod, kTimeout);

        // Firmware & API
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus,framePeriod,kTimeout);

        // MotionProfile Targets for Auxiliary PID1
        // not used
        // mFXMotor.setStatusFramePeriod(StatusFrame.Status_17_Targets1,framePeriod,kTimeout);

        // for (int i=0; i<framePeriods.length; i++){
        //     do{
        //         double period = FXMotor.getStatusFramePeriod(framePeriods[i]);
        //         error = FXMotor.getLastError();
        //         System.out.println("statusFramePeriod for "+framePeriods[i].toString()+" is "+period +" with "+error.toString());
                
        //         Timer.delay(.001);
        //     } while(!error.equals(ErrorCode.OK));

        // }


        System.out.println("setFramePeriods took "+(Timer.getFPGATimestamp()-start));
    }

    public static double getStatorCurrent(TalonFX FXMotor){
        return getRoutine(FXMotor, GetRoutine.STATORCURRENT);
    }

    public static double getSelectedSensorPosition(TalonFX FXMotor){
        return getRoutine(FXMotor, GetRoutine.SENSORPOSITION);
    }

    public static double getFirmwareVersion(TalonFX motor){
        return getRoutine(motor, GetRoutine.FIRMWAREVERSION);
    }


    public static void configStatorCurrentLimitPermanent(TalonFX motor, StatorCurrentLimitConfiguration config) {
        final int kRetries = 5;
        double retries = kRetries;
        ErrorCode error = ErrorCode.OK;
        double start = Timer.getFPGATimestamp();
        String cmd = "configStatorCurrentLimit";
        do {
            if (retries != kRetries){
                Timer.delay(.005);
                System.out.println("Failed ("+error.toString()+") to "+cmd+"() will retry");
            }

            error = motor.configStatorCurrentLimit(config);
            error = motor.getLastError();

        } while (!error.equals(ErrorCode.OK) && retries-->0);
        if (error.equals(ErrorCode.OK)){
            // System.out.println("Successfully completed "+cmd+"(), time needed "+(Timer.getFPGATimestamp()-start));
        }
        else{
            System.out.println("Failed to "+cmd+"(), time needed "+(Timer.getFPGATimestamp()-start));
        }
    }

    public static void setSelectedSensorPositionVolatile(TalonFX motor, double sensorPosition) {
        final int kRetries = 5;
        double retries = kRetries;
        ErrorCode error = ErrorCode.OK;
        double start = Timer.getFPGATimestamp();
        String cmd = "setSelectedSensorPosition";
        do {
            if (retries != kRetries){
                Timer.delay(.005);
                System.out.println("Failed ("+error.toString()+") to "+cmd+"() will retry");
            }

            error = motor.setSelectedSensorPosition(sensorPosition);
            error = motor.getLastError();

        } while (!error.equals(ErrorCode.OK) && retries-->0);
        if (error.equals(ErrorCode.OK)){
            // System.out.println("Successfully completed "+cmd+"(), time needed "+(Timer.getFPGATimestamp()-start));
        }
        else{
            System.out.println("Failed to "+cmd+"(), time needed "+(Timer.getFPGATimestamp()-start));
        }
    }

    public static void setNeutralModeVolatile(TalonFX motor, NeutralMode mode) {
        final int kRetries = 5;
        double retries = kRetries;
        ErrorCode error = ErrorCode.OK;
        double start = Timer.getFPGATimestamp();
        String cmd = "setNeutralMode";
        do {
            if (retries != kRetries){
                Timer.delay(.010);
                System.out.println("Failed ("+error.toString()+") to "+cmd+"() will retry");
            }

            motor.setNeutralMode(mode);
            error = motor.getLastError();

        } while (!error.equals(ErrorCode.OK) && retries-->0);
        if (error.equals(ErrorCode.OK)){
            // System.out.println("Successfully completed "+cmd+"(), time needed "+(Timer.getFPGATimestamp()-start));
        }
        else{
            System.out.println("Failed to "+cmd+"(), time needed "+(Timer.getFPGATimestamp()-start));
        }
    }

    public static void setInvertedVolatile(TalonFX motor) {
        final int kRetries = 5;
        double retries = kRetries;
        ErrorCode error = ErrorCode.OK;
        double start = Timer.getFPGATimestamp();
        String cmd = "setInverted";
        do {
            if (retries != kRetries){
                Timer.delay(.010);
                System.out.println("Failed ("+error.toString()+") to "+cmd+"() will retry");
            }

            motor.setInverted(true); // coast is default so only brake is needed
            error = motor.getLastError();

        } while (!error.equals(ErrorCode.OK) && retries-->0);
        if (error.equals(ErrorCode.OK)){
            // System.out.println("Successfully completed "+cmd+"(), time needed "+(Timer.getFPGATimestamp()-start));
        }
        else{
            System.out.println("Failed to "+cmd+"(), time needed "+(Timer.getFPGATimestamp()-start));
        }
    }

    public static void configFactoryDefaultPermanent(TalonFX motor) {

        final int kRetries = 5;
        double retries = kRetries;
        ErrorCode error = ErrorCode.OK;
        double start = Timer.getFPGATimestamp();
        String cmd = "configFactoryDefault";
        do {
            if (retries != kRetries){
                Timer.delay(.010);
                System.out.println("Failed ("+error.toString()+") to "+cmd+"() will retry");
            }

            motor.configFactoryDefault();
            error = motor.getLastError();

        } while (!error.equals(ErrorCode.OK) && retries-->0);
        if (error.equals(ErrorCode.OK)){
            // System.out.println("Successfully completed "+cmd+"(), time needed "+(Timer.getFPGATimestamp()-start));
        }
        else{
            System.out.println("Failed to "+cmd+"(), time needed "+(Timer.getFPGATimestamp()-start));
        }
    }
}