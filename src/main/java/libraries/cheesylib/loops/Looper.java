package libraries.cheesylib.loops;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List object. They are started when the robot
 * powers up and stopped after the match.
 */
public class Looper implements ILooper {
    ILooper mSubLooper;

    // public Looper(double period) {
    //     mSubLooper = (ILooper) new LooperOrig(period);
    // }

    public Looper(double period, int priority) {
        mSubLooper = (ILooper) new Looper2(period,priority);
    }

    @Override
    public synchronized int register(Loop loop) {
        return mSubLooper.register(loop);
    }

    public synchronized void start() {
        mSubLooper.start();
    }

    public synchronized void stop() {
        mSubLooper.stop();
    }

    public void outputToSmartDashboard() {
        mSubLooper.outputToSmartDashboard();
    }
}
