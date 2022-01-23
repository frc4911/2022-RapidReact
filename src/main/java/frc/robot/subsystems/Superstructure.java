package frc.robot.subsystems;

import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;

public class Superstructure extends Subsystem{
    


    private static String sClassName;
    private static int sInstanceCount;
    private static Superstructure sInstance = null;
    public  static Superstructure getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new Superstructure(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    // Looping methods for subsystem
    private Loop mLoop = new Loop(){
        
        @Override
        public void onStart(Phase phase){

        }

        @Override
        public void onLoop(double timestamp){

        }

        @Override
        public void onStop(double timestamp){

        }

    };

    // Handling methods

    // State transfers

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+" getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Superstructure(String caller){
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    @Override
    public String getLogHeaders() {
        // TODO Auto-generated method stub
        return "Superstructure";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        // TODO Auto-generated method stub
        return "Superstructure.Values";
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        
    }

}
