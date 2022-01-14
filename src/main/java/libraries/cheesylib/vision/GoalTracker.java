package libraries.cheesylib.vision;

import java.util.ArrayList;
import java.util.Comparator;
//import java.util.Iterator;
import java.util.List;

import libraries.cheesylib.geometry.Pose2d;

/**
 * This is used in the event that multiple goals are detected to judge all goals based on timestamp, stability, and
 * continuation of previous goals (i.e. if a goal was detected earlier and has changed locations). This allows the robot
 * to make consistent decisions about which goal to aim at and to smooth out jitter from vibration of the camera.
 *
 * @see GoalTrack
 */
public class GoalTracker {
    /**
     * Track reports contain all of the relevant information about a given goal track.
     */
    public static class TrackReport {
        // Transform from the field frame to the vision target.
        public Pose2d field_to_target;

        // The timestamp of the latest time that the goal has been observed
        public double latest_timestamp;

        // The percentage of the goal tracking time during which this goal has
        // been observed (0 to 1)
        public double stability;

        // The track id
        public int id;

        public TrackReport(GoalTrack track) {
            this.field_to_target = track.getSmoothedPosition();
            this.latest_timestamp = track.getLatestTimestamp();
            this.stability = track.getStability();
            this.id = track.getId();
        }
    }

    // This came from Constants.java and should be initialized in Robot.java
    public static double maxGoalTrackAge = 2.5;

    /**
     * TrackReportComparators are used in the case that multiple tracks are active (e.g. we see or have recently seen
     * multiple goals). They contain heuristics used to pick which track we should aim at by calculating a score for
     * each track (highest score wins).
     */
    public static class TrackReportComparator implements Comparator<TrackReport> {
        // Reward tracks for being more stable (seen in more frames)
        double mStabilityWeight;
        // Reward tracks for being recently observed
        double mAgeWeight;
        double mCurrentTimestamp;
        // Reward tracks for being continuations of tracks that we are already
        // tracking
        double mSwitchingWeight;
        int mLastTrackId;

        public TrackReportComparator(double stability_weight, double age_weight, double switching_weight,
                                     int last_track_id, double current_timestamp) {
            this.mStabilityWeight = stability_weight;
            this.mAgeWeight = age_weight;
            this.mSwitchingWeight = switching_weight;
            this.mLastTrackId = last_track_id;
            this.mCurrentTimestamp = current_timestamp;
        }

        double score(TrackReport report) {
            double stability_score = mStabilityWeight * report.stability;
            double age_score = mAgeWeight
                    * Math.max(0, (maxGoalTrackAge - (mCurrentTimestamp - report.latest_timestamp))
                    / maxGoalTrackAge);
            double switching_score = (report.id == mLastTrackId ? mSwitchingWeight : 0);
            return stability_score + age_score + switching_score;
        }

        @Override
        public int compare(TrackReport o1, TrackReport o2) {
            double diff = score(o1) - score(o2);
            // Greater than 0 if o1 is better than o2
            if (diff < 0) {
                return 1;
            } else if (diff > 0) {
                return -1;
            } else {
                return 0;
            }
        }
    }

    List<GoalTrack> mCurrentTracks = new ArrayList<>();
    int mNextId = 0;

    public GoalTracker() {
	}

    public synchronized void reset() {
        mCurrentTracks.clear();
    }

    double xTarget = 0.0;
    double acceptableError = 6.0;
    boolean useXTarget = false;
    public void setXTarget(double x, double error){
        xTarget = x;
        acceptableError = error;
        useXTarget = true;
    }
    public void enableXTarget(boolean enable){
        useXTarget = enable;
    }

    public synchronized void update(double timestamp, List<Pose2d> field_to_goals) {
        // make new tracks to accommadate for each observation
        if (mCurrentTracks.size() < field_to_goals.size()) {
            for (int i = mCurrentTracks.size(); i < field_to_goals.size(); i++) {
                mCurrentTracks.add(GoalTrack.makeNewTrack(timestamp, field_to_goals.get(i), mNextId));
                ++mNextId;
            }
        }

        for (int i = 0; i < mCurrentTracks.size(); i++) {
            // if there as an observation to update the track
            if (field_to_goals.size() > i) {
                // try to update the given track
                mCurrentTracks.get(i).tryUpdate(timestamp, field_to_goals.get(i));

                // if dead after update, force update to reset the target position
                if (!mCurrentTracks.get(i).isAlive()) {
                    mCurrentTracks.get(i).forceUpdate(timestamp, field_to_goals.get(i));
                }
        
                // if no observation for an update empty update 
            } else {
                mCurrentTracks.get(i).emptyUpdate();
            }
        }
    }

    public synchronized boolean hasTracks() {
        return !mCurrentTracks.isEmpty();
    }

    public synchronized List<TrackReport> getTracks() {
        List<TrackReport> rv = new ArrayList<>();
        for (GoalTrack track : mCurrentTracks) {
            rv.add(new TrackReport(track));
        }
        return rv;
    }

    public synchronized void clearTracks(){
        mCurrentTracks = new ArrayList<>();
    }
}