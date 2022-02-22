package frc.robot.autos;

import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.util.CrashTrackingRunnable;

/**
 * This class selects, runs, and stops (if necessary) a specified autonomous
 * mode.
 */
public class AutoModeExecutor {
    private AutoModeBase mAutoMode;
    private Thread mThread = null;

    public void setAutoMode(AutoModeBase new_auto_mode) {
        mAutoMode = new_auto_mode;
    }

    public void start() {
        if (mThread == null) {
            mThread = new Thread(new CrashTrackingRunnable() {
                @Override
                public void runCrashTracked() {
                    if (mAutoMode != null) {
                        mAutoMode.run();
                    }
                }
            });

            mThread.start();
        }

    }

    public void stop() {
        if (mAutoMode != null) {
            mAutoMode.stop();
        }

        mThread = null;
    }

    public AutoModeBase getAutoMode() {
        return mAutoMode;
    }

}