package libraries.cyberlib.utils;

public class TestValueTracker{
    private double max = Double.MIN_VALUE;
    private double ave = 0;
    private double min = Double.MAX_VALUE;
    private double last = Double.NaN;
    private String description;
    private double processCount = 0;
 
    public TestValueTracker(String description){
        this.description = description;
    }

    public void process(double newValue){
        if (newValue > max){
            max = newValue;
        }

        if (newValue < min){
            min = newValue;
        }
        ave += newValue;
        processCount++;
        last = newValue;
    }

    public void process(boolean newValue){
        if (newValue){
            max = 1;
        }

        if (!newValue){
            min = 0;
        }
        ave += newValue?1:0;
        processCount++;
        last = newValue?1:0;
    }

    public void dumpMax (double exp, double range){
        dumpOne (max, exp, range, description+" (max)");
    }

    public void dumpMin (double exp, double range){
        dumpOne (min, exp, range, description+" (min)");
    }

    public void dumpAve (double exp, double range){
        if (processCount>0){
            ave /= processCount;
        }
        dumpOne (ave, exp, range, description+" (ave)");
    }

    public void dumpLast (double exp, double range){
        dumpOne (last, exp, range, description+" (last)");
    }

    public void dumpChange (double exp, double range){
        dumpOne (max-min, exp, range, description+" (change)");
    }

    private void dumpOne(double value, double expected, double range, String desc){
        String results = "FAILED";

        value = myRound(value,10);

        if ((value <= (expected + range)) && (value >= (expected - range))){
            results = "passed";
        }
        String expectedStr = "______(" + expected + " +- " + range + ") " + results;
        System.out.println(desc + " = " + value + expectedStr);
    }

    private double myRound (double value, double factor){
        return Math.round(value*factor)/factor;
    }
}