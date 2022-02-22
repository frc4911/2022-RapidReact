package libraries.cyberlib.control;

import libraries.cheesylib.geometry.Translation2d;

public class HolonomicFeedforward {
    private final SwerveDriveFeedforwardGains forwardGains;
    private final SwerveDriveFeedforwardGains strafeGains;

    public HolonomicFeedforward(SwerveDriveFeedforwardGains forwardGains,
                                SwerveDriveFeedforwardGains strafeGains) {
        this.forwardGains = forwardGains;
        this.strafeGains = strafeGains;
    }

    public HolonomicFeedforward(SwerveDriveFeedforwardGains translationGains) {
        this(translationGains, translationGains);
    }

    public Translation2d calculateFeedforward(Translation2d velocity, Translation2d acceleration) {
        // We don't use `DrivetrainFeedforwardConstants.calculateFeedforward` because we want to apply kS (the static
        // constant) proportionally based on the rest of the feedforwards.

        double forwardFeedforward = forwardGains.getVelocityConstant() * velocity.x();
        forwardFeedforward += forwardGains.getAccelerationConstant() * acceleration.x();

        double strafeFeedforward = strafeGains.getVelocityConstant() * velocity.y();
        strafeFeedforward += strafeGains.getAccelerationConstant() * acceleration.y();

        Translation2d feedforwardVector = new Translation2d(forwardFeedforward, strafeFeedforward);

        // Apply the kS constant proportionally to the forward and strafe feedforwards based on their relative
        // magnitudes
        Translation2d feedforwardUnitVector = feedforwardVector.normalize();
        forwardFeedforward += Math.copySign(feedforwardUnitVector.x() * forwardGains.getStaticConstant(),
                forwardFeedforward);
        strafeFeedforward += Math.copySign(feedforwardUnitVector.y() * strafeGains.getStaticConstant(),
                strafeFeedforward);

        return new Translation2d(forwardFeedforward, strafeFeedforward);
    }

    public SwerveDriveFeedforwardGains getForwardGains() {
        return forwardGains;
    }

    public SwerveDriveFeedforwardGains getStrafeGains() {
        return strafeGains;
    }

}
