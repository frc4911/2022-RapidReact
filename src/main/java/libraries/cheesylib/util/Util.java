package libraries.cheesylib.util;

import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util {
    public static final double kEpsilon = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {}

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    /**
     * Limits a value between a minimum and a maximum value.
     *
     * @param v The value to limit.
     * @param min The minimum value of the range. This value must be less than max.
     * @param max The maximum value of the range. This value must be greater than min.
     * @return the limited value.
    */
    public static double limit(double v, double min, double max) {
        if (min > max) {
            throw new IllegalArgumentException("min must not be greater than max");
        }

        return Math.min(max, Math.max(min, v));
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    /**
     * Checks if the given input is within the range (min, max), both exclusive.
     */
    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static double deadBand(double val, double deadband){
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    public static double getLineAngle(final List<Double> p1, final List<Double> p2) {
        final double angle = Math.atan2(p2.get(1) - p1.get(1), p2.get(0) - p1.get(0));
        return Util.normalize_angle_positive(angle);
    }

    public static double normalize_angle_positive(final double angle) {
        return (angle % (2.0 * Math.PI) + 2.0 * Math.PI) % (2.0 * Math.PI);
    }

    public static double normalize_angle(final  double angle) {
        var a = normalize_angle_positive(angle);
        if (a > Math.PI) {
            a -= 2.0 * Math.PI;
        }

        return a;
    }

    public static double shortest_angular_distance(final double from_angle, final double to_angle) {
        return normalize_angle(to_angle - from_angle);
    }
}
