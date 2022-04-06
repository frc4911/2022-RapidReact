// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.autos.AutoModeSelector;
import frc.robot.config.RobotConfiguration;
import frc.robot.constants.Constants;
import frc.robot.limelight.LimelightManager;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.*;
import libraries.cheesylib.autos.AutoModeBase;
import libraries.cheesylib.autos.AutoModeExecutor;
import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.loops.Looper;
import libraries.cheesylib.subsystems.SubsystemManager;
import libraries.cheesylib.util.CrashTracker;
import libraries.cyberlib.utils.RobotName;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    // The constructor for RobotName looks in the deploy directory
    // (home/lvuser/deploy/) on the robot for
    // a file name RobotName.txt. If found it reads the first line of the file and
    // saves what it reads as the robot's name. Note: the name passed into the
    // constructor
    // is the name used if no RobotName.txt file is found.
    // in the src\main\deploy directory of VS is a RobotName.txt file
    // this file is downloaded to the deploy directory with each deploy of the robot
    // jar file
    RobotName robotName = new RobotName("2022Robot");

    private String mClassName;

    // Subsystems
    private SubsystemManager mSubsystemManager;
    private Superstructure mSuperstructure;
    private JSticks mJSticks;
    private Swerve mSwerve;
    private Shooter mShooter;
    private Indexer mIndexer;
    private Climber mClimber;
    private Collector mCollector;
    private RobotStateEstimator mRobotStateEstimator;
    private Limelight mLimelight;
    private LEDCanifier mLEDCanifier;

    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
    private AutoModeExecutor mAutoModeExecutor;
    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private final double mLoopPeriod = .005;
    private Looper mSubsystemLooper = new Looper(mLoopPeriod, Thread.NORM_PRIORITY + 1);

    @Override
    public void robotInit() {
        System.out.println("robotInit() begins");
        Timer.delay(.05); // give rest of system 50 msec to come up first
        mClassName = this.getClass().getSimpleName();
        // LiveWindow.disableAllTelemetry();
        // LiveWindow.setEnabled(false);
        // Initializing subsystems
        mSubsystemManager = SubsystemManager.getInstance(mClassName);
        mJSticks = JSticks.getInstance(mClassName);
        mSuperstructure = Superstructure.getInstance(mClassName);
        mSwerve = Swerve.getInstance(mClassName);
        mShooter = Shooter.getInstance(mClassName);
        mIndexer = Indexer.getInstance(mClassName);
        mClimber = Climber.getInstance(mClassName);
        mCollector = Collector.getInstance(mClassName);
        mRobotStateEstimator = RobotStateEstimator.getInstance(mClassName);
        mLEDCanifier = LEDCanifier.getInstance(mClassName);
        mLimelight = new Limelight(
                RobotConfiguration.getRobotConfiguration(RobotName.name).getLimelightConfiguration(),
                Constants.kLowRes1xZoom);

        // Create subsystem manager and add all subsystems it will manage
        mSubsystemManager = SubsystemManager.getInstance(mClassName);
        mSubsystemManager.initializeSubsystemManager((int) (mLoopPeriod * 1000),
                Arrays.asList(
                        // List of subsystems
                        mJSticks,
                        mSuperstructure,
                        mSwerve,
                        mShooter,
                        mIndexer,
                        mClimber,
                        mCollector,
                        mLimelight,
                        mLEDCanifier,
                        mRobotStateEstimator));

        // ask each subsystem to register itself
        mSubsystemManager.registerEnabledLoops(mSubsystemLooper);

        if (mSwerve != null) {
            mSwerve.zeroSensors();
            mSwerve.zeroSensors(new Pose2d());

            // robotState.feignVisionTargets();
            // mSwerve.startTracking(Constants.kDiskTargetHeight, new Translation2d(-6.0,
            // 0.0), true, new Rotation2d());
            mSwerve.stop();

            mAutoModeSelector = new AutoModeSelector();
            mAutoModeSelector.updateModeCreator();

            // Always generate trajectories when robot code starts
            mTrajectoryGenerator.generateTrajectories(mSwerve.mSwerveConfiguration.trajectoryConfig);
        }

        LimelightManager.getInstance().setLimelight(mLimelight);

        System.out.println("RobotInit() ends");
    }

    @Override
    public void robotPeriodic() {
        mAutoModeSelector.outputToSmartDashboard();
        // Ensure timely updates when graphing values etc.
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void autonomousInit() {
        System.out.println("*********** AutonomousInit() begins ***********");
        try {
            autoConfig();

            mSubsystemLooper.stop();
            mSubsystemLooper.start();

            mAutoModeExecutor.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
        System.out.println("AutonomousInit() ends");
    }

    public void autoConfig() {
        if (mSwerve != null) {
            mSwerve.zeroSensors();
            mSwerve.zeroSensors(new Pose2d());
            // mSwerve.setNominalDriveOutput(0.0);
            // mSwerve.requireModuleConfiguration();
            // mSwerve.set10VoltRotationMode(true);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        System.out.println("*********** TeleopInit() begins ***********");
        try {
            mSubsystemLooper.stop();

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mSubsystemLooper.start();
            teleopConfig();
            // robotState.enableXTarget(false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
        System.out.println("TeleopInit() ends");
    }

    public void teleopConfig() {
        // if (mSwerve != null) {
        // mSwerve.setNominalDriveOutput(0.0);
        // mSwerve.set10VoltRotationMode(false);
        // }
    }

    @Override
    public void teleopPeriodic() {
        // This will send the Network Table data to DriveStation at a consistent rate.
    }

    @Override
    public void disabledInit() {
        System.out.println("*********** disabledInit() begins ***********");
        try {
            System.gc();

            mSubsystemLooper.stop();

            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mSubsystemLooper.start();
            LimelightManager.getInstance().writePeriodicOutputs();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
        System.out.println("disabledInit() ends");
    }

    @Override
    public void disabledPeriodic() {
        try {
            // Update auto modes
            mAutoModeSelector.updateModeCreator();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        System.out.println("*********** testInit() begins ***********");
        mSubsystemLooper.stop();
        mSubsystemLooper.start();
        System.out.println("testInit() ends");
    }

    @Override
    public void testPeriodic() {
    }
}
