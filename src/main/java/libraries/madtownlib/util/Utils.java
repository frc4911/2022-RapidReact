package libraries.madtownlib.util;

//import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Utils {
    public static double normalize(double current, double test){
    	if(current > test) return current;
    	return test;
    }
    
    public static double boundAngleNeg180to180Degrees(double angle){
        // Naive algorithm
        while(angle >= 180.0) {angle -= 360.0;}
        while(angle < -180.0) {angle += 360.0;}
        return angle;
    }
    
    public static double boundAngle0to360Degrees(double angle){
        // Naive algorithm
        while(angle >= 360.0) {angle -= 360.0;}
        while(angle < 0.0) {angle += 360.0;}
        return angle;
    }
    
    public static double boundToScope(double scopeFloor, double scopeCeiling, double argument){
    	double stepSize = scopeCeiling - scopeFloor;
    	while(argument >= scopeCeiling) {argument -= stepSize;}
    	while(argument < scopeFloor) {argument += stepSize;}
    	return argument;
    }
    
    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle){
    	double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if(lowerOffset >= 0){
        	lowerBound = scopeReference - lowerOffset;
        	upperBound = scopeReference + (360 - lowerOffset);
        }else{
        	upperBound = scopeReference - lowerOffset; 
        	lowerBound = scopeReference - (360 + lowerOffset);
        }
        while(newAngle < lowerBound){
        	newAngle += 360; 
        }
        while(newAngle > upperBound){
        	newAngle -= 360; 
        }
        if(newAngle - scopeReference > 180){
        	newAngle -= 360;
        }else if(newAngle - scopeReference < -180){
        	newAngle += 360;
        }
        return newAngle;
    }
    
    public static boolean shouldReverse(double goalAngle, double currentAngle){
    	goalAngle = boundAngle0to360Degrees(goalAngle);
    	currentAngle = boundAngle0to360Degrees(currentAngle);
    	double reversedAngle = boundAngle0to360Degrees(currentAngle + 180);
    	double angleDifference = Math.abs(goalAngle - currentAngle);
    	double reversedAngleDifference = Math.abs(goalAngle - reversedAngle);
    	angleDifference = (angleDifference > 180) ? 360-angleDifference : angleDifference;
    	reversedAngleDifference = (reversedAngleDifference > 180) ? 360-reversedAngleDifference : reversedAngleDifference;
    	return reversedAngleDifference < angleDifference;
    }
}
