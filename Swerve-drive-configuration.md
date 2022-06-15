# 2022 Swerve Drive Configuration

Stand robot on edge so:
1. the back of the robot is on the floor and the front points up to the ceiling	
2. the underneath (wheels) are easily visible 
3. it is convenient for you or a partner to see the lights of the motors. 

This doc assumes that the development pc and the drivestation PC are the same. Connect the drive station with USB cable if possible to allow simultaneous wifi to internet

This step is optional. The intent of this step is to remove any code that may inadvertently cause issue. This is optional because the Phoenix explorer is designed to coexist with the robot code. In my experience they have coexisted fine. 
Start by connecting to the Linux file system on the robot. This can be done with any ftp client (like filezilla) or more easily with the Windows File Explorer (it’s icon is a manilla folder)
1. In the file explorer connect to the robot by typing the following into the “quick access” text box
   1. "ftp://roborio-4911-frc.local/"
   2. if that doesn't work try ftp://172.22.11.2 if connected over USB or ftp://10.49.11.2 if connected over wifi
   3. Leave the user name and password blank
2. goto home/lvuser/ and delete any .jar files that are present. If code has been downloaded to the robot previously there should be a .jar file with the name of the github repository. The typical size is 6 to 14MBs
3. Power cycle the robot so it stops running the current software

Mark the robot so it is obvious which module is which and what direction is positive. I prefer a permanent marker on the chassis (so the info is readily available whenever the robot is being worked on (but this is optional)
1. Modules are numbered 0 to 3 in this order frontRight, frontLeft, backLeft, backRight
   1. Front is shootwards, back is opposite, left and right are determined when looking at the robot from above and behind (not below). The module numbers should increase in a CCW direction (from the top)
2. Wheel rotation is positive in the CCW direction when viewed from above (which means it is CW from the bottom)
3. Wheels align front to back when at 0 degrees. The bevel/gear on the side of the wheel should be on the left side of all wheels when those wheels point forward (a.k.a. Pointing 0 degrees)

Assign CAN ID’s to the motors and the cancoders using the phoenix tuner

Install CTRE’s PhoenixTuner on the PC and start it up. On the “robot controller install” tab select the “172.22.11.2 # RoboRIO Over USB” option if connected over USB

Or enter 10.49.11.2 if connected over wifi 

The port is 1250 for both

Hit the “Run Temporary Diagnostic Server” button. If everything is good the following text should appear in the window

```
Connecting to roboRIO... (02/10/2022 1:40 PM)
Connected sucessfully.
Writing files...
Written file: /home/lvuser/PhoenixDiagnosticsProgram (PhoenixDiagnosticsProgram)
Written file: /tmp/frcdebug (frcdebug)
Written file: /home/lvuser/robotDebugCommand (robotDebugCommand)
Starting Temporary Diagnostics Server
. /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t 2> /dev/null @ /home/lvuser
Updating File Write Permissions
Syncing filesystem to ensure files are on the flash
Starting Temporary Diagnostics Server
. /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r 2> /dev/null @ /home/lvuser

Duration: 00:00:04.50
```

After switching to the “CAN Devices” a list of devices on the CAN bus should appear (sometimes this takes a few seconds)

Set can ids for motors, cancoders and pigeon
Reset all to factory default
Test motors to verify they work and run full speed to get max velocity and make sure a positive .set moves the motor in the desired direction and make sure the encoders count up

Measure ticks per revolution by hand for all motors and cancoder

Make sure pigeon counts up when the robot turns CCW

Set robotname file in VS

Set wheel dimensions in swerveconfiguration

Update other constants


When starting up it expect 0 degrees is straight ahead.
- Positive x is forward
- Positive y is left
- Angles use CCW convention i.e. angles increase in the CCW directions

definitions:
- Front is the way the robot shoots (shootwards)
- Back is opposite of front (collectwards)
- Right and Left are defined with the robot on the ground (looking forward from above). This is important because the robot is often viewed from underneath. It is leaned against a wall during development.

The modules are numbered as follows:
- 0 -  front right
- 1 - front left
- 2 - back left
- 3 - back right

front right being 0 is arbitrary

the increasing order is CCW because that is positive angle direction

The motor CAN ID's for our robots will be as follows:
- front right drive - 0
- front left drive - 1
- back left drive - 2
- back right drive - 3
- front right steer - 4
- front left steer - 5
- back left steer - 6
- back right steer - 7

In the past the CAN ID started at 1 and incremented up but experience has shown that this requires extra thought when mapping from module to drive motor. I now think that starting at 0 is best so the drive motor's CAN ID's match the module number

The cancoder CAN id's are as follows:
- front right - 0
- front left - 1
- back left - 2
- back right - 3

this is arbitrary, the ID just matches the module number

When a positive value is passed to the steering motor i.e. `mSteerMotor.set(PercentOutput,1)`. The motor rotates in the CCW direction when viewed from above (or CW from underneath). The lights on the motor should flash green (driving forward) and the encoder count should increase. The corresponding CANCODER should also count up.

When a module wheel is pointing 0 degrees and a positive value is passed to the drive motor the robot moves forward. The lights on the drive motor are green and the encoder counts up.

I propose that we use SI units in our code and switch to/from English units at the edges (prints and telemetry)
external <-> internal
------------------------
degrees <-> radians
inches <-> meters

The imu should increase when the robot is turned CCW.

This section describes manual driving of our robot.

### Quick Overview
- Swerve robots have four wheels that can be operated independently of each other. Each wheel can be steered (pointed) in any direction and driven at any (possible) speed. This document describes the software that coordinates these wheel actions to enable the robot to drive and rotate simultaneously. 
- Each of the four wheels has two motors. One for rotation of the wheel around a vertical axis to point the wheel in any direction. The other to rotate the wheel about its axis to make the wheel roll. The wheel and it’s associated two motors is called a module. 
- Additionally, the robot has a single IMU to enable the robot to know which direction the robot is pointing. This is called the robots heading. It is used to enable field relative driving which enables the robot to move in the direction desired regardless of the robots current heading.
- Finally, the robot is controlled by a joystick. This document assumes a single “classic” joystick traditionally developed for flight simulation. It has a single stick (~6” long) that is gripped with one hand. Many teams use XBox style controllers. The same principles apply but the mapping and physical movements will differ.
- The goal of the code is to make the joystick movements cause the robot to move in an intuitive way. If the the joystick is pushed forward the robot moves forward. If it is pushed to the left the robot moves to the left. And if the joystick is twisted to the left the robot twists to the left.

### Code Overview
The methods are presented as much as is possible to follow the flow of inputs from the joystick to the output to the motors.

`jSticks.java` constructor gets an instance of the single SwerveHeadingController. The headingController does the following:
- Allows the setting and getting of the current state of twist control i.e. OFF, MAINTAIN (there is also SNAP  which doesn’t appear to be used)
- Allows setting the current goal heading
- Creates a PID from supplied kd, ki, and kp constants
- Provides an “update” method that returns a PID value based on the difference of the goal to the current heading

`jSticks.java - public void readPeriodicInputs()`
- This method is called by the scheduler every 20 to 100 msec. There is no benefit in calling it more frequently than the joystick values are refreshed
- Reads the x, y, and z values from the joystick(s)
  - Y is flipped (*=-1) because pushing the joystick forward returns negative values 
  - X is flipped (*=-1) because pushing the joystick to the left returns negative values
  - Z is flipped (*=-1) because twisting CCW (from above) returns negative values
  - Joystick return values can be viewed in the DriverStation on the USB/Joystick tab
  - Buttons are also checked to switch to robot centric driving and to reset the IMU

`jSticks.java public void teleopRoutines()`
- This is called by a handler routine that is called from `onLoop()` it is called on the same frequency and just after readPeriodicInputs
- The x and y values are switched because the joystick y-axis (forward and back) corresponds to the x-axis and the joystick x-axis corresponds to the y-axis in our coordinate system
- The robot tends to twist slightly as it drives even if no twist input comes from the joystick. To minimize this behaviour the heading of the imu is monitored and if twist is detected w/o joystick twist then an opposite twist is assigned to the z value so the rest of the code twists back and the robot drives straighter. If the user purposely introduces twist then the code does not modify the z value but lets the user control the twist.
- So in this method there is code that tracks whether the user has not input twist for the last .2 seconds. 
- If true it remembers the current heading and uses the heading controller PID to get the correction twist. This done in the call to Swerve.setTeleopValues(x, y, z (or corrected z), lowPower (never used), fieldOriented, true if z is corrected)

`Swerve.java` constructor creates an instance of the pigeon (a.k.a. IMU) and then creates each module. It passes in the constants for each module based on the robot name that is saved in a file that is downloaded when the jar file is deployed.

```java 
mPigeon = Pigeon.getInstance();
if (RobotName.name.equals(Constants.kJuniorName)) {
    mSwerveConfiguration = Constants.kSwerveConfigurationJunior;
    mModules.add(mFrontRight = new SwerveDriveModule(Constants.kFrontRightModuleConstantsJunior, mSwerveConfiguration.maxSpeedInMetersPerSecond));
    mModules.add(mFrontLeft = new SwerveDriveModule(Constants.kFrontLeftModuleConstantsJunior, mSwerveConfiguration.maxSpeedInMetersPerSecond));
    mModules.add(mBackLeft = new SwerveDriveModule(Constants.kBackLeftModuleConstantsJunior, mSwerveConfiguration.maxSpeedInMetersPerSecond));
    mModules.add(mBackRight = new SwerveDriveModule(Constants.kBackRightModuleConstantsJunior, mSwerveConfiguration.maxSpeedInMetersPerSecond));
} else if (RobotName.name.equals(Constants.kDeadEyeName)) {
… 
}
```

Swerve’s constructor goes on to create the SwerveDriveHelper (which is not currently used), the kinematics instance, and the Odometry instance.
```java 
mSwerveDriveHelper = new SwerveDriveHelper(mSwerveConfiguration.maxSpeedInMetersPerSecond,
            mSwerveConfiguration.maxSpeedInRadiansPerSecond);

mKinematics = new SwerveDriveKinematics(mSwerveConfiguration.moduleLocations);
    mOdometry = new SwerveDriveOdometry(mKinematics, mPigeon.getYaw());
mPose = mOdometry.getPose();
```

The SwerveDriveHelper (which is not currently used) is from MadTown and it’s goal is improve the driver’s experience controlling the robot during the match. It does the following: 
- Snaps the translational inputs to cardinal points on the compass if they are already within a tolerance. It has different cardinal points if in robot centric or field centric mode
- It has it’s own deadband for translational input
- It scales the translational inputs differently based on the low_power flag
- It scales the rotational inputs differently based on if it is using the raw or corrected z

`Swerve.java`

```java 
public void setTeleopInputs(double forward, double strafe, double rotation, boolean low_power, boolean field_relative, boolean use_heading_controller) {
    if (mControlState != ControlState.MANUAL) {
        mControlState = ControlState.MANUAL;
    }
    mPeriodicIO.forward = forward;
    mPeriodicIO.strafe = strafe;
    mPeriodicIO.rotation = rotation;
    mPeriodicIO.low_power = low_power;
    mPeriodicIO.field_relative = field_relative;
    mPeriodicIO.use_heading_controller = use_heading_controller;
}
```

Simply saves the values for later use

Swerve now runs as a regular subsystem. It’s readPeriodicInputs saves the current heading and calls the each of the SwerveDriveModules->readPeriodicInputs method.

```java 
public synchronized void readPeriodicInputs() {
    mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeon.getYaw().getDegrees()).rotateBy(mGyroOffset);

    // read modules
    mModules.forEach((m) -> m.readPeriodicInputs());
}
```

`Swerve.writePeriodicOutput` sets the state for each module and calls their writePeriodicOutput method

```java 
public synchronized void writePeriodicOutputs() {
    // Set the module state for each module
    // All modes should use this method of module states.
    for (int i = 0; i < mModules.size(); i++) {
        mModules.get(i).setState(mPeriodicIO.swerveModuleStates[i]);
    }
    // System.out.println(mPeriodicIO.swerveModuleStates[0].toString());
    mModules.forEach((m) -> m.writePeriodicOutputs());
}
```

`Swerve.onLoop() updateOdometry()` this method gets the current pose

```java 
private void updateOdometry(double timestamp) {
    var frontRight = mFrontRight.getState();
    var frontLeft = mFrontLeft.getState();
    var backLeft = mBackLeft.getState();
    var backRight = mBackRight.getState();

    // brian it would be nice to order the modules CCW like the rest of the code
    mChassisSpeeds = mKinematics.toChassisSpeeds(frontLeft, frontRight, backLeft, backRight);
    mPose = mOdometry.updateWithTime(timestamp, getAngle(), frontLeft, frontRight, backLeft, backRight);
}
```

SwerveDriveModule
```java 
public synchronized SwerveModuleState getState() {
    // Convert encoder readings to SI units
    double steerAngleInRadians = encoderUnitsToRadians(mPeriodicIO.steerPosition);
    steerAngleInRadians %= 2.0 * Math.PI;
    if (steerAngleInRadians < 0.0) {
        steerAngleInRadians += 2.0 * Math.PI;
    }

    return new SwerveModuleState(encVelocityToMetersPerSecond(mPeriodicIO.driveDemand),
            Rotation2d.fromRadians(steerAngleInRadians));
}
```
