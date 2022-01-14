package libraries.cyberlib.utils;

public class CyberMath {
    public static double cRound(double num, double digits){
        return Math.round(num*digits) / digits;
    }

    public static double cTrunc(double num, double digits){
        return ((int)(num*digits)) / digits;
    }
}