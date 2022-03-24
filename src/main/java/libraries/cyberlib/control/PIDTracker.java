package libraries.cyberlib.control;

import edu.wpi.first.wpilibj.Timer;

public class PIDTracker {
    
    // must move before stopped
    // final summary
    // max velocity
    // max acc/dec
    // passed through goal
    // entered from below/above
    // exited below/above

    private int size;
    private int target;
    private int targetZone;
    private int targetNearEdge;
    private int targetFarEdge;
    private int startPosition;
    private int pathLength;
    private int moveThreshold;
    private double percentTraveled;
    private int directionOfTravel;

    private int[] positions;
    private double[] timestamps; // seconds
    private double[] velocities;
    private double maxAveVelocity;
    private double maxAveAcc;
    private double minAveVelocity;
    private double minAveAcc;
    private double currentAveVelocity;
    private double currentAveAcc;

    private int positionCount;
    private boolean hasMoved;
    private boolean isStopped;
    private int index;
    private int previousIndex;
    private boolean inTargetZoneNow;
    private boolean wasInTargetZone;
    private int targetZoneNearEdgeCrossed;
    private int targetZoneFarEdgeCrossed;

    public PIDTracker(int target, int targetZone, int ptsPerMovingAverage, int moveThreshold){

        this.size = ptsPerMovingAverage;
        this.target = target;
        this.moveThreshold = moveThreshold;
        targetZone /= 2;

        positions = new int[size];
        timestamps = new double[size];
        velocities = new double[size];
        previousIndex =  size-1;
    }

    public void addPosition(int position){

        if (positionCount == 0){
            startPosition = position;
            directionOfTravel = startPosition <= target ? 1 : -1;

            startPosition *= directionOfTravel;
            target *= directionOfTravel;
            position *= directionOfTravel;

            pathLength = target-startPosition;
            targetNearEdge = target-targetZone;
            targetFarEdge = target+targetZone;
        }
        else{
            position *= directionOfTravel;
        }

        if (Math.abs(position - startPosition)>moveThreshold){
            hasMoved = true;
        }

        percentTraveled = ((double)(position-startPosition))/(double)pathLength;

        // total points
        positionCount++;
        
        // index is always valid and it now points to oldest position
        positions[index] = position;
        timestamps[index] = Timer.getFPGATimestamp();

        // index now points to most recent position
        // prevIndex points to oldest position

        // where are we now
        if (position >= targetNearEdge && position <= targetFarEdge){
            wasInTargetZone = true;
            inTargetZoneNow = true;
        }
        else{
            inTargetZoneNow = false;
        }

        // need at least two points to analyze more 
        if (positionCount > 1){
            // where was the previous position?
            // int previousIndex = index-1;
            // previousIndex = previousIndex<0 ? size-1 : previousIndex;

            // get the previous position
            double prevPosition = positions[previousIndex];

            // track target zone entrance/exit
            if (prevPosition < targetNearEdge && position >= targetNearEdge){
                targetZoneNearEdgeCrossed++;
            }
            else if (prevPosition >= targetNearEdge && position < targetNearEdge){
                targetZoneNearEdgeCrossed++;
            }

            if (prevPosition < targetFarEdge && position >= targetFarEdge){
                targetZoneFarEdgeCrossed++;
            }
            else if (prevPosition >= targetFarEdge && position < targetFarEdge){
                targetZoneFarEdgeCrossed++;
            }

            if (positionCount > 1){
                currentAveVelocity = (positions[index]-positions[previousIndex])/(timestamps[index]-timestamps[previousIndex]);
                maxAveVelocity = Math.max(maxAveVelocity, currentAveVelocity);
                minAveVelocity = Math.min(minAveVelocity, currentAveVelocity);

                velocities[index] = currentAveVelocity;
            }

            if (positionCount > 2){
                currentAveAcc = (velocities[index]-velocities[previousIndex])/(timestamps[index]-timestamps[previousIndex]);
                maxAveAcc = Math.max(maxAveAcc,currentAveAcc);
                minAveAcc = Math.min(minAveAcc,currentAveAcc);
            }

            if (Math.abs(position-prevPosition)<moveThreshold){
                isStopped = true;
            }
            else{
                isStopped = false;
            }

            previousIndex = index;
            index = (++index)%positions.size;
        }
    }

    public boolean isStopped(){
        return isStopped;
    }

    public boolean hasMoved(){
        return hasMoved;
    }

    public boolean inTargetZoneNow(){
        return inTargetZoneNow;
    }

    public boolean wasInTargetZone(){
        return wasInTargetZone;
    }

    public double getMaxAverageVelocity(){
        return maxAveVelocity;
    }
    public double getMinAverageVelocity(){
        return minAveVelocity;
    }
    public double getMaxAverageAcc(){
        return maxAveAcc;
    }
    public double getMinAverageAcc(){
        return minAveAcc;
    }
    public double getCurrentAverageVelocity(){
        return currentAveVelocity;
    }
    public double getCurrentAverageAcc(){
        return currentAveVelocity;
    }
}
