package libraries.cyberlib.control;

public class SwerveDriveFeedforwardGains {
    private final double velocityConstant;
    private final double accelerationConstant;
    private final double staticConstant;

    public SwerveDriveFeedforwardGains(double velocityGain, double accelerationGain, double staticGain) {
        this.velocityConstant = velocityGain;
        this.accelerationConstant = accelerationGain;
        this.staticConstant = staticGain;
    }

    public double calculateFeedforward(double velocity, double acceleration) {
        double feedforward = velocityConstant * velocity;
        feedforward += accelerationConstant * acceleration;

        feedforward += Math.copySign(staticConstant, feedforward);

        return feedforward;
    }

    public double getVelocityConstant() {
        return velocityConstant;
    }

    public double getAccelerationConstant() {
        return accelerationConstant;
    }

    public double getStaticConstant() {
        return staticConstant;
    }

}
