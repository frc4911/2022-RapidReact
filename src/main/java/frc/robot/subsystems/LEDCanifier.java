package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;

import libraries.cheesylib.subsystems.Subsystem;

/**
 * Provides an LED controller using a CTR Canifier.
 */
public class LEDCanifier extends Subsystem {
    private CANifier mCanifier;
    private PeriodicOutputs mPeriodicOutputs;
    private boolean mOutputsChanged = true;

    private static String sClassName;
    private static int sInstanceCount;
    private static LEDCanifier sInstance = null;
    public  static LEDCanifier getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new LEDCanifier(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private LEDCanifier(String caller) {
        sClassName        = this.getClass().getSimpleName();
        printUsage(caller);
        mCanifier = new CANifier(0);
        mCanifier.configFactoryDefault(100);
 
        mPeriodicOutputs = new PeriodicOutputs();

        // Force a first update.
        mOutputsChanged = true;
    }

    // public int getDeviceId() {
    //     return mCanifier.getDeviceID();
    // }

    public synchronized void setLEDColor(double red, double green, double blue) {
        if (red != mPeriodicOutputs.red || green != mPeriodicOutputs.green || blue != mPeriodicOutputs.blue) {
            mPeriodicOutputs.red = red;
            mPeriodicOutputs.green = green;
            mPeriodicOutputs.blue = blue;
            mOutputsChanged = true;
        }
    }

    @Override
    public String getLogHeaders() {
        return "";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        return "";
    }

    @Override
    public synchronized void readPeriodicInputs() {
        // A: Blue
        // B: Green
        // C: Red
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // A: Blue
        // B: Green
        // C: Red
        if (mOutputsChanged) {
            mCanifier.setLEDOutput(mPeriodicOutputs.green, CANifier.LEDChannel.LEDChannelA);
            mCanifier.setLEDOutput(mPeriodicOutputs.red, CANifier.LEDChannel.LEDChannelB);
            mCanifier.setLEDOutput(mPeriodicOutputs.blue, CANifier.LEDChannel.LEDChannelC);
            mOutputsChanged = false;
        }
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {
        mPeriodicOutputs = new PeriodicOutputs();
        mOutputsChanged = true;
        writePeriodicOutputs();
    }

    @Override
    public void zeroSensors() {
    }

    private static class PeriodicOutputs {
        public double red;
        public double green;
        public double blue;
    }
}
