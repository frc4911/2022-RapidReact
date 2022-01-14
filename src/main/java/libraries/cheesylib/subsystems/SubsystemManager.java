package libraries.cheesylib.subsystems;

import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.loops.Looper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {
    private List<Subsystem> mAllSubsystems;
    private List<Loop> mLoops = new ArrayList<>();
    private SubsystemLogManager mSSLogMngr;
    private int mSSCount;
    private TheLoop mTheLoop;
    private String[] mSSEmptyLog;
    private int mLoopPeriod;

    private static String sClassName;
    private static int sInstanceCount;
    private static SubsystemManager sInstance = null;
    public  static SubsystemManager getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new SubsystemManager(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private SubsystemManager(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
    }

    public void initializeSubsystemManager(int loopPeriod, List<Subsystem> allSubsystems) {
        mLoopPeriod = loopPeriod;
        mAllSubsystems = allSubsystems;
        mSSCount = mAllSubsystems.size();
        mTheLoop = new TheLoop();
        mSSLogMngr = new SubsystemLogManager();
        mSSEmptyLog = new String[mSSCount];
    }

    // enabling and disabling the logger is handled in the logger
    // to simplify the rest of the code it assumes logging is always enabled
    private void prepAllSubsystemsForLogging(boolean disableLogging) {
        mSSLogMngr.startLogging(disableLogging);
        int i=0;
        mSSLogMngr.addToLine("LineNum,LoopNum,TicToc,Time,Bitmask");
        for (Subsystem s : mAllSubsystems) {
            String header = s.getLogHeaders();
            // count commas
            int columns = header.length() - header.replace(",", "").length();
            // create string of just commas
            mSSEmptyLog[i++] = ",".repeat(columns);
            mSSLogMngr.addToLine(header);
        }

        mSSLogMngr.endLine();
    }

    private int runALoop(int index){
        Subsystem s = mAllSubsystems.get(index);
        s.readPeriodicInputs();
        mLoops.get(index).onLoop(Timer.getFPGATimestamp());
        s.writePeriodicOutputs();
        mSSLogMngr.addToLine(s.getLogValues(false));
        return s.whenRunAgain();
    }

    // subsystems call these two methods when they want to wake up or run soon
    public void scheduleMe(int listIndex, int when){
        scheduleMe(listIndex, when, true);
    }

    public void scheduleMe(int listIndex, int when, boolean clearSchedule){
        mTheLoop.scheduleMe(listIndex, when, clearSchedule);
    }

    private class TheLoop implements Loop {
        private final int lSchedLength = 1000;  // 1000 msec == one second, cannot schedule beyond one sec ahead
        private final int lInitialDelay = 10;
        private int[] lSchedule = new int[lSchedLength];
        private long lTicToc;
        private int lLineNum;
        private Phase lPhase;
        private long lLostTime;
    
        @Override
        public void onStart(Phase thePhase) {
            // thePhase is bogus, get the real one below
            boolean disableLogging = true; //Matthew (3/34)

            lPhase = getPhase();
            mLoops.forEach(l -> l.onStart(lPhase));

            // clear schedule of any leftover bits
            for (int i=0; i<lSchedule.length; i++){
                lSchedule[i] = 0;
            }
            
            // get current time and turn it into MSec's
            lTicToc = (long)(Timer.getFPGATimestamp()*1000.0+.5); // System.currentTimeMillis();

            // schedule all subsystems to start, spread out 
            for (int i=0; i<mSSCount; i++){
                lSchedule[(int)((lTicToc+lInitialDelay+i*2)%lSchedule.length)] = 1<<i;
            }
            lLineNum=0;

            // when disabled a log file is not wanted
            if (lPhase == Phase.DISABLED){
                disableLogging = true;
            }
            prepAllSubsystemsForLogging(disableLogging);
       }

        @Override
        public void onLoop(double timestamp) {

            // each time this is called it loops mLoopPeriod times
            // mLoopPeriod is the number msecs between calls
            // one loop per msec
            for (int i=0; i<mLoopPeriod; i++){
                int subsystemIndex = 0;
                int bitmask = lSchedule[(int)(lTicToc%lSchedule.length)];

                mSSLogMngr.addToLine(""+lLineNum++ +","+i+","+lTicToc+","+Timer.getFPGATimestamp()+","+bitmask);

                // each bit represents a SS to run now
                while (bitmask>0){
                    // turn bit into index into all subsystem list
                    while ((bitmask & (1 << subsystemIndex)) == 0){
                        mSSLogMngr.addToLine(mSSEmptyLog[subsystemIndex]);
                        subsystemIndex++;
                    }

                    // run all phases for this SS and get next time to run in delta MSec
                    int nextRun = runALoop(subsystemIndex);

                    // 0 means do not schedule
                    if (nextRun != 0){
                        lSchedule[(int)((lTicToc+nextRun)%lSchedule.length)] |= (1 << subsystemIndex);
                    }

                    // clear bit for SS just run
                    bitmask &= ~(1 << subsystemIndex);
                    // point to next SS
                    subsystemIndex++;
                }
                // handle completely empty line and end of partial line 
                for (int j=subsystemIndex; j<mSSCount; j++){
                    mSSLogMngr.addToLine(mSSEmptyLog[subsystemIndex]);
                }
                mSSLogMngr.endLine();

                // clear this schedule
                lSchedule[(int)(lTicToc%lSchedule.length)] = 0;
                // advance to next schedule slot
                lTicToc++;

                // every 250 msec handle telemetry, call logging and telemetry
                if (lTicToc%250 == 0){
                    mSSLogMngr.addToLine(""+lLineNum++ +",,"+lTicToc+","+Timer.getFPGATimestamp()+",-1");
                    for (Subsystem s : mAllSubsystems) {
                        mSSLogMngr.addToLine(s.getLogValues(true));
                        s.outputTelemetry();
                    }
                    mSSLogMngr.endLine();
                }

                // these lines are only needed if mLoopPeriod is greater than the actual period
                // currently they are equal
                // if mLoopPeriod is greater then this stops the loop from getting ahead
                lLostTime = ((long)(Timer.getFPGATimestamp()*1000.0+.5))-lTicToc;
                if ( lLostTime <= 0){
                    break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            mLoops.forEach(l -> l.onStop(timestamp));
            System.out.println("schedule time lost "+lLostTime);
        }

        public void scheduleMe(int listIndex, int when, boolean clearScheduled){
            if (clearScheduled){
                int mask = ~(1<<listIndex);
                for (int i=0; i<lSchedLength; i++){
                    lSchedule[i] &= mask;
                }
            }
            lSchedule[(int)((lTicToc+when)%lSchedule.length)] |= (1 << listIndex);
        }

        // This could be passed in
        private Phase getPhase(){
            if(DriverStation.isEnabled()){
                if (DriverStation.isTeleop()){
                    return Phase.TELEOP;
                }
                else if (DriverStation.isAutonomous()){
                    return Phase.AUTONOMOUS;
                }
                else if (DriverStation.isTest()){
                    return Phase.TEST;
                }
            }
            return Phase.DISABLED;
        }
    }

    // ask all subsystems to register themselves then register with the looper
    public void registerEnabledLoops(Looper looper2) {
        mAllSubsystems.forEach(s -> s.registerEnabledLoops(this));
        looper2.register(mTheLoop);
    }

    // each subsystem calls this to register its loop and is returned its scheduling slot
    @Override
    public int register(Loop loop) {
        mLoops.add(loop);
        return mLoops.size()-1;
    }

    // unused
    @Override
    public void start(){}

    // unused
    @Override
    public void outputToSmartDashboard() {}
 
    // unused
    @Override
    public void stop() {}
 }
