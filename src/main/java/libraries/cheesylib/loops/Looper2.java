package libraries.cheesylib.loops;

import libraries.cheesylib.util.CrashTrackingRunnable;

import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List
 * object. They are started when the robot powers up and stopped after the
 * match.
 */
public class Looper2 implements ILooper {
    public double kPeriod = 0.05;
    private int mThreadPriority;
    private Thread mThread;
    private boolean mRunning;
    private final List<Loop> mLoops;
    private final Object mObject = new Object();

    public Looper2(double period) {
        this(period,Thread.NORM_PRIORITY);
    }

    public Looper2(double period, int priority) {
        mThreadPriority = priority;
        kPeriod = period;
        mRunning = false;
        mLoops = new ArrayList<>();

        createLooper();
    }

    @Override
    public synchronized int register(Loop loop) {
        synchronized (mObject) {
            mLoops.add(loop);
            return mLoops.size()-1;
        }
    }

    public synchronized void start() {
        if (!mRunning) {
            System.out.println("Starting loops");

            synchronized (mObject) {
                for (Loop loop : mLoops) {
                    loop.onStart(null);
                }
                mRunning = true;
                mObject.notify();
            }
        }
    }

    public synchronized void stop() {
        if (mRunning) {
            System.out.println("Stopping loops");

            mRunning = false;
            synchronized (mObject) {
                double timestamp = Timer.getFPGATimestamp();    
                for (Loop loop : mLoops) {
                    loop.onStop(timestamp);
                }
            }
        }
    }

    // not used
    public void outputToSmartDashboard() {
    }

    // create thread to regularly run onLoop
    private void createLooper() {
        mThreadPriority = Math.max(Math.min(Thread.MAX_PRIORITY, mThreadPriority),Thread.MIN_PRIORITY);

        mThread = new Thread(new CrashTrackingRunnable() {
            @Override
            public void runCrashTracked() {
                // hold the lock while running
                synchronized (mObject) {
                    double now = 0;
                    int counter = 0;
                    int loopOverruns = 0;
                    Thread thread = Thread.currentThread(); 
                    thread.setPriority(mThreadPriority);
                    while (true) {
                        try {
                            if (mRunning) {
                                now = Timer.getFPGATimestamp();
        
                                counter++;
                                for (Loop loop : mLoops) {
                                    loop.onLoop(now);
                                }
        
                                // sleepTime is time in seconds from now (start of this loop) to desired start of next loop
                                double sleepTime = now+kPeriod-Timer.getFPGATimestamp();

                                // if sleepTime is positive then convert to an MSec (an integer) and sleep
                                if (sleepTime > 0){
                                    // convert to MSec (adding .5 turns the truncate into a roundoff)
                                    // sleep holds the lock
                                    Thread.sleep((int)(sleepTime*1000.0+.5));
                                }
                                else {
                                    // ran long so reschedule after giving others a small chance
                                    // sleep holds the lock
                                    loopOverruns++;
                                    Thread.sleep(1); // may want to sleep(0) instead or not at all
                                }
                            } 
                            else {
                                // this releases the lock while waiting for notify
                                if (counter>0){
                                    System.out.println("loop overruns "+loopOverruns+" total loops "+counter);
                                }
                                mObject.wait();
                                now = 0;
                                counter = 0;
                                loopOverruns = 0;
                            }
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                            Thread.currentThread().interrupt();
                        }
                    }
                }
            }
        });
        mThread.setPriority(mThreadPriority);
        mThread.start();
    }
}
