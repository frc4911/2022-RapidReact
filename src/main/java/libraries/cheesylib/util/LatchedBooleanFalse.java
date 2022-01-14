package libraries.cheesylib.util;

/**
 * An iterative boolean latch.
 * <p>
 * Returns true once if and only if the value of newValue changes from false to true.
 */
public class LatchedBooleanFalse {
    private boolean mLast = true;

    public boolean update(boolean newValue) {
        boolean ret = false;
        if (!newValue && mLast) {
            ret = true;
        }
        mLast = newValue;
        return ret;
    }
}