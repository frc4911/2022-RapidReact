package libraries.cheesylib.util;

/**
 * An iterative int latch.
 * <p>
 * Returns true once if and only if the value of newValue changes from the desired value.
 */
public class LatchedIntFalse {
    private int mLast = Integer.MAX_VALUE;

    public boolean update(int newValue, int desiredValue) {
        boolean ret = false;
        if ((newValue != desiredValue) && (desiredValue == mLast)) {
            ret = true;
        }
        mLast = newValue;
        return ret;
    }
}