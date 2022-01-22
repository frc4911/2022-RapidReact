// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.JSticks;
import frc.robot.subsystems.Superstructure;
import libraries.cheesylib.loops.Looper;
import libraries.cheesylib.subsystems.SubsystemManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private String mClassName;

  // Subsystems
  private SubsystemManager mSubsystemManager;
  private Superstructure   mSuperstructure;
  private JSticks          mJSticks;

  private final double mLoopPeriod = .005;
  private Looper mSubsystemLooper = new Looper(mLoopPeriod,Thread.NORM_PRIORITY+1);

  @Override
  public void robotInit() {
    //Initializing subsystems
    mClassName = this.getClass().getSimpleName();
    mSubsystemManager = SubsystemManager.getInstance(mClassName);
    mSuperstructure = Superstructure.getInstance(mClassName);
    mJSticks = JSticks.getInstance(mClassName);

    //Create subsystem manager and add all subsystems it will manage
    mSubsystemManager = SubsystemManager.getInstance(mClassName);
		mSubsystemManager.initializeSubsystemManager( (int)(mLoopPeriod*1000),
        Arrays.asList(
          //List of subsystems
          mSuperstructure,
          mJSticks
        )
    );

    // ask each subsystem to register itself
		mSubsystemManager.registerEnabledLoops(mSubsystemLooper);

  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
