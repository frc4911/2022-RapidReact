package libraries.cyberlib.trajectory.swerve;

public class OptimizationParameters {
    double x = 0.0;
    double y = 0.0;
    double elongation = 0.5;
    double rs = 0.0;
    double re = 0.0;
    double theta_elongation = 0.0;
    public double theta_lambda = 0.0;

    public OptimizationParameters() {
        this(0.0, 0.0, 0.5, 0.0, 0.0, 1.0, 0.0);
    }

    public OptimizationParameters(
            double x,
            double y,
            double elongation,
            double rs,
            double re,
            double theta_elongation,
            double theta_lambda) {
        this.x = x;
        this.y = y;
        this.elongation = elongation;
        this.rs = rs;
        this.re = re;
        this.theta_elongation = theta_elongation;
        this.theta_lambda = theta_lambda;
    }

    public OptimizationParameters(OptimizationParameters other) {
        this(other.x, other.y, other.elongation, other.rs, other.re, other.theta_elongation, other.theta_lambda);
    }

    public int size() {
        return 5;
    }

    public double get(int position) {
        if (position == 0)
           return this.elongation;
        if (position == 1)
            return this.rs;
        if (position == 2)
            return this.re;
        if (position == 3)
            return this.theta_elongation;
        if (position == 4)
            return this.theta_lambda;

        throw new ArrayIndexOutOfBoundsException(
                String.format("invalid index %d. Provide a value between 0 and 4", position));
    }

    public void set(int position, double value) {
        if (position == 0) {
            this.elongation = value;
            return;
        }

        if (position == 1) {
            this.rs = value;
            return;
        }

        if (position == 2) {
            this.re = value;
            return;
        }

        if (position == 3) {
            this.theta_elongation = value;
            return;
        }

        if (position == 4) {
            this.theta_lambda = value;
            return;
        }

        throw new ArrayIndexOutOfBoundsException("Provide a value between 0 and 4");
    }

    @Override
    public String toString() {
        return String.format("elongation={%.3f}, ", elongation) +
                String.format("re={%.3f}, ", re) +
                String.format("rs={%.3f}, ", rs) +
                String.format("theta.elongation={%.3f}, ", theta_elongation) +
                String.format("theta.lambda={%.3f}, ", theta_lambda);
    }
}
