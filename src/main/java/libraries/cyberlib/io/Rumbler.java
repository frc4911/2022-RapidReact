package libraries.cyberlib.io;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;

// TODO: create more rumble patterns

// This class implements rumble. It is created
// for all joysticks but it only works for those 
// that support rumble.
// This class extends GenericHID which requires the slot
// number for its constructor.
// This means that Rumbler cannot be created until the
// slot is known
public class Rumbler extends GenericHID {

    private boolean rumbling = false;

    public Rumbler(int slot) {
        super(slot);
    }

    // if not currently rumbling then create rumble class and start it
    public void rumble(final double rumblesPerSecond, final double numberOfSeconds) {
        if (!rumbling) {
            final RumbleThread r = new RumbleThread(rumblesPerSecond, numberOfSeconds);
            r.start();
        }
    }

    public boolean isRumbling() {
        return rumbling;
    }

    // self contained class that does the rumbling
    public class RumbleThread extends Thread {
        public double rumblesPerSec = 1;
        public long interval = 500;
        public double seconds = 1;
        public double startTime = 0;

        public RumbleThread(final double rumblesPerSecond, final double numberOfSeconds) {
            rumblesPerSec = rumblesPerSecond;
            seconds = numberOfSeconds;
            interval = (long) (1 / (rumblesPerSec * 2) * 1000);
        }

        public void run() {
            rumbling = true;
            startTime = Timer.getFPGATimestamp();
            try {
                while ((Timer.getFPGATimestamp() - startTime) < seconds) {
                    setRumble(RumbleType.kLeftRumble, 1);
                    setRumble(RumbleType.kRightRumble, 1);
                    sleep(interval);
                    setRumble(RumbleType.kLeftRumble, 0);
                    setRumble(RumbleType.kRightRumble, 0);
                    sleep(interval);
                }
            } catch (final InterruptedException e) {
                rumbling = false;
                e.printStackTrace();
            }
            rumbling = false;
        }
    }
}