package libraries.cyberlib.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TimeCheck {
    double longCounter = 0;
    double checkCounter = 0;
    double lastTime;
    double threshhold;
    String dashboardKey;

    public TimeCheck(double threshhold, String dashboardKey) {
        this.threshhold = threshhold;
        lastTime = Timer.getFPGATimestamp();
        this.dashboardKey = dashboardKey;
    }

    public void checkTime() {
        double now = Timer.getFPGATimestamp();
        double delta = now - lastTime;
        lastTime = now;
        checkCounter++;

        if (delta > threshhold) {
            longCounter++;
            int percent = (int) Math.round((longCounter / checkCounter) * 100.0);
            String msg = longCounter + "/" + checkCounter + "=" + percent + "% (" + Math.round(delta * 1000.0) / 1000.0
                    + ")";
            // System.out.println(msg+" "+str);
            SmartDashboard.putString(dashboardKey, msg);
        }
    }

}