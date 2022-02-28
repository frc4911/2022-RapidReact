package libraries.cyberlib.control;

public final class PidGains {
    public final double p, i, d;

    public PidGains(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }
}
