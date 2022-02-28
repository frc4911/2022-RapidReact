package libraries.cyberlib.io;

import edu.wpi.first.wpilibj.Joystick;
import libraries.cheesylib.util.CrashTrackingRunnable;

// TODO: add rescan

// a simple static class that finds all of the joysticks
// it runs at startup
// as each joystick is created in robot.java they
// call findThisJoystick which looks for the 
// joystick name in the list of found joysticks
// if it finds a match it marks the slot as used and
// returns the index
public class JoystickFinder {
    // Logitech
    // Controller (Gamepad F310)

    // Logitech
    // Logitech Extreme 3D

    // Logitech
    // Logitech Attack 3

    // Thrustmaster
    // T.16000M

    // Turnigy
    // Evolution PRO

    // Xbox
    // Controller (Xbox One For Windows)

    // joystick names go in each slot as they are discovered
    // if a match is found then the name is modified to reserve it
    // an empty string means: check again

    // joysticks can/will be plugged in after startup
    // so all slots are checked each time
    private String[] mSlotName = { "", "", "", "", "", "" };
    private final String mAllocated = " allocated";
    private Joystick[] mAllJoysticks = new Joystick[mSlotName.length];
    private Thread mThread;
    private int mThreadPriority = Thread.NORM_PRIORITY - 1;
    private Object mObject = new Object();

    // private static JoystickFinder mInstance = new JoystickFinder();
    // public static JoystickFinder getInstance(){
    // return mInstance;
    // }
    private static String sClassName;
    private static int sInstanceCount;
    private static JoystickFinder sInstance = null;

    public static JoystickFinder getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new JoystickFinder(caller);
        } else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller) {
        System.out.println("(" + caller + ") " + "getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private JoystickFinder(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);

        for (int i = 0; i < mAllJoysticks.length; i++) {
            mAllJoysticks[i] = new Joystick(i);
            mSlotName[i] = mAllJoysticks[i].getName();
        }

        createJoystickFinder();
        printSticks();
    }

    private void printSticks() {
        // print what was found
        for (int i = 0; i < mSlotName.length; i++) {
            if (mSlotName[i].length() == 0) {
                System.out.println(i + ": <empty>");
            } else {
                System.out.println(i + ": " + mSlotName[i]);
            }
        }
    }

    // Joysticks are found in the order they are
    // asked for. First request gets the first match found
    // searches start in slot 0 and move up
    public Joystick findThisJoystick(String name) {
        // System.out.println("findThisJoystick: "+name+" "+System.currentTimeMillis());
        for (int i = 0; i < mSlotName.length; i++) {
            if (mSlotName[i].equals(name)) {
                // found a match
                // save a modified name
                // this will protect this joystick from being reallocated
                // if a second identical joystick is searched for
                mSlotName[i] = name + mAllocated;
                return mAllJoysticks[i];
            }
        }
        synchronized (mObject) {
            mObject.notifyAll();
        }
        return null;
    }

    private void lookForNewJoystick() {
        // check empty slots to see if a stick appeared
        for (int i = 0; i < mSlotName.length; i++) {
            if (mSlotName[i].length() == 0) {
                mSlotName[i] = mAllJoysticks[i].getName();
            }
        }
    }

    // create thread to look for joystick in background
    private void createJoystickFinder() {

        mThread = new Thread(new CrashTrackingRunnable() {
            @Override
            public void runCrashTracked() {
                while (true) {
                    try {
                        synchronized (mObject) {
                            mObject.wait();
                        }
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    lookForNewJoystick();
                }
            }
        });
        mThread.setPriority(mThreadPriority);
        mThread.start();
    }
}