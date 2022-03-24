package frc.robot.states;

public interface TimedLEDState {
    public static final double kDefaultBlinkDuration = 0.500; // In sec
    public static final double kFastBlinkDuration = 0.100; // In sec

    void getCurrentLEDState(LEDState desiredState, double timestamp);

    class BlinkingLEDState implements TimedLEDState {
        public static BlinkingLEDState kIndexerLoaded = new BlinkingLEDState(
                LEDState.kShoot, // ready to shoot when indexer is loaded
                LEDState.kOff,
                kDefaultBlinkDuration);

        public static BlinkingLEDState kIndexerUnjamming = new BlinkingLEDState(
                LEDState.kIndexerLoaded,
                LEDState.kOff,
                kFastBlinkDuration);

        public static BlinkingLEDState kCollectorDeployed = new BlinkingLEDState(
                LEDState.kCollectorDeployed,
                LEDState.kOff,
                kDefaultBlinkDuration);

        public static BlinkingLEDState kCollectorUnjamming = new BlinkingLEDState(
                LEDState.kCollectorDeployed,
                LEDState.kOff,
                kFastBlinkDuration);

        public static BlinkingLEDState kShoot = new BlinkingLEDState(
                LEDState.kShoot,
                LEDState.kOff,
                kDefaultBlinkDuration);

        public static BlinkingLEDState kAiming = new BlinkingLEDState(
                LEDState.kShoot,
                LEDState.kOff,
                kFastBlinkDuration);

        public static BlinkingLEDState kDrive = new BlinkingLEDState(
                LEDState.kDrive,
                LEDState.kOff,
                kDefaultBlinkDuration);

        LEDState mStateOne = new LEDState(0.0, 0.0, 0.0);
        LEDState mStateTwo = new LEDState(0.0, 0.0, 0.0);
        double mDuration;

        public BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double duration) {
            mStateOne.copyFrom(stateOne);
            mStateTwo.copyFrom(stateTwo);
            mDuration = duration;
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            if ((int) (timestamp / mDuration) % 2 == 0) {
                desiredState.copyFrom(mStateOne);
            } else {
                desiredState.copyFrom(mStateTwo);
            }
        }
    }

    class StaticLEDState implements TimedLEDState {
        public static StaticLEDState kStaticOff = new StaticLEDState(LEDState.kOff);
        public static StaticLEDState kCollectorDeployed = new StaticLEDState(LEDState.kCollectorDeployed);
        public static StaticLEDState kShoot = new StaticLEDState(LEDState.kShoot);
        public static StaticLEDState kIndexerLoaded = new StaticLEDState(LEDState.kIndexerLoaded);
        public static StaticLEDState kDrive = new StaticLEDState(LEDState.kDrive);
        public static StaticLEDState kAutoStart = new StaticLEDState(LEDState.kAutoStart);
        public static StaticLEDState kRobotZeroed = new StaticLEDState(LEDState.kRobotZeroed);

        LEDState mStaticState = new LEDState(0.0, 0.0, 0.0);

        public StaticLEDState(LEDState staticState) {
            mStaticState.copyFrom(staticState);
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            desiredState.copyFrom(mStaticState);
        }
    }
}
