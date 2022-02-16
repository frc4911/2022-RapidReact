package libraries.cyberlib.kinematics;

import libraries.cheesylib.geometry.Pose2d;
import libraries.cheesylib.geometry.Rotation2d;
import libraries.cheesylib.geometry.Translation2d;
import libraries.cyberlib.utils.Angles;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

class SwerveDriveOdometryTest {
  private final Translation2d m_fl = new Translation2d(12, 12);
  private final Translation2d m_fr = new Translation2d(12, -12);
  private final Translation2d m_bl = new Translation2d(-12, 12);
  private final Translation2d m_br = new Translation2d(-12, -12);

  private final SwerveDriveKinematics m_kinematics =
          new SwerveDriveKinematics(m_fl, m_fr, m_bl, m_br);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
          new Rotation2d());

  @Test
  void testTwoIterations() {
    // 5 units/sec  in the x axis (forward)
    final SwerveModuleState[] wheelSpeeds = {
            new SwerveModuleState(5, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(5, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(5, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(5, Rotation2d.fromDegrees(0))
    };

    m_odometry.updateWithTime(0.0, new Rotation2d(),
            new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState());
    var pose = m_odometry.updateWithTime(0.10, new Rotation2d(), wheelSpeeds);

    assertAll(
            () -> assertEquals(5.0 / 10.0, pose.getTranslation().x(), 0.01),
            () -> assertEquals(0, pose.getTranslation().y(), 0.01),
            () -> assertEquals(0.0, pose.getRotation().getDegrees(), 0.01)
    );
  }

  @Test
  void test90degreeTurn() {
    // This is a 90 degree turn about the point between front left and rear left wheels
    //        Module 0: speed 18.84955592153876 angle 90.0
    //        Module 1: speed 42.14888838624436 angle 26.565051177077986
    //        Module 2: speed 18.84955592153876 angle -90.0
    //        Module 3: speed 42.14888838624436 angle -26.565051177077986

    final SwerveModuleState[] wheelSpeeds = {
            new SwerveModuleState(18.85, Rotation2d.fromDegrees(90.0)),
            new SwerveModuleState(42.15, Rotation2d.fromDegrees(26.565)),
            new SwerveModuleState(18.85, Rotation2d.fromDegrees(-90)),
            new SwerveModuleState(42.15, Rotation2d.fromDegrees(-26.565))
    };
    final var zero = new SwerveModuleState();

    m_odometry.updateWithTime(0.0, new Rotation2d(), zero, zero, zero, zero);
    final var pose = m_odometry.updateWithTime(1.0, Rotation2d.fromDegrees(90.0), wheelSpeeds);

    assertAll(
            () -> assertEquals(12.0, pose.getTranslation().x(), 0.01),
            () -> assertEquals(12.0, pose.getTranslation().y(), 0.01),
            () -> assertEquals(90.0, pose.getRotation().getDegrees(), 0.01)
    );
  }

  @Test
  void testGyroAngleReset() {
    var gyro = Rotation2d.fromDegrees(90.0);
    var fieldAngle = Rotation2d.fromDegrees(0.0);
    m_odometry.resetPosition(new Pose2d(new Translation2d(), fieldAngle), gyro);
    var state = new SwerveModuleState();
    m_odometry.updateWithTime(0.0, gyro, state, state, state, state);
    state = new SwerveModuleState(1.0, Rotation2d.fromDegrees(0));
    var pose = m_odometry.updateWithTime(1.0, gyro, state, state, state, state);

    assertAll(
            () -> assertEquals(1.0, pose.getTranslation().x(), 0.1),
            () -> assertEquals(0.00, pose.getTranslation().y(), 0.1),
            () -> assertEquals(0.00, pose.getRotation().getRadians(), 0.1)
    );
  }

  @Test
  void TestSwerveOdometry_StraightLineWithRotation() {
    // Test inverse kinematics while translating forward and rotating
    // Drive forward 1 m/s for 5 s while rotating 8 degrees /s

    final var omega_speed = Math.toRadians(8);
    final double max_speed = 3.5;
    final double cruise_speed = 1.0;
    final int frequency = 50;
    final double deltaTime = 1.0 / frequency; // loop at 20 ms
    final var deltaAngle = Rotation2d.fromDegrees(Math.toDegrees(omega_speed) * deltaTime);

    var gyro = Rotation2d.identity();  // 0 degrees is straight ahead

    for (double currentTime = 0; currentTime < 5; currentTime += deltaTime) {
      // Command the robot to move forward while taking gyro into affect
      var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(cruise_speed, 0.0, omega_speed, gyro);

      // Perform inverse kinematics to get the module states that represent the desired chassis speeds
      var states = m_kinematics.toSwerveModuleStates(chassisSpeeds,  Translation2d.identity());

      // Normalize model states to what is attainable
      SwerveDriveKinematics.desaturateWheelSpeeds(states, max_speed);

      // In a real robot these states would be passed into the robot to execute.
      // Additionally, the robot would construct module states used to update the odometry.
      // NOTE: Getting the current module states would be done before passing in the new ones.

      // Now update odometry
      var position = m_odometry.updateWithTime(currentTime, gyro, states);
      System.out.printf("time: %.2f - %s%n", currentTime, position.toString());

      // update for next loop iteration
      gyro = gyro.rotateBy(deltaAngle);
    }

    System.out.printf(m_odometry.getPose().toString());

    assertEquals(5.0, m_odometry.getPose().getTranslation().x(), 0.01);
  }

  @Test
  public void TestOdometryWithSwerveModuleOptimize() {
    final SwerveModuleState[] wheelSpeeds = {
            new SwerveModuleState(5, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(5, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(5, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(5, Rotation2d.fromDegrees(0))
    };

    m_odometry.updateWithTime(0.0, new Rotation2d(),
            new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState());
    var pose = m_odometry.updateWithTime(0.10, new Rotation2d(), wheelSpeeds);

    assertAll(
            () -> assertEquals(5.0 / 10.0, pose.getTranslation().x(), 0.01),
            () -> assertEquals(0, pose.getTranslation().y(), 0.01),
            () -> assertEquals(0.0, pose.getRotation().getDegrees(), 0.01)
    );

    // Current heading is 0 degrees but desired motion is backwards
    var angleB = Rotation2d.fromDegrees(0);
    SwerveModuleState[] optimizedB = {
            SwerveModuleState.optimize(new SwerveModuleState(5, Rotation2d.fromDegrees(180)), angleB),
            SwerveModuleState.optimize(new SwerveModuleState(5, Rotation2d.fromDegrees(180)), angleB),
            SwerveModuleState.optimize(new SwerveModuleState(5, Rotation2d.fromDegrees(180)), angleB),
            SwerveModuleState.optimize(new SwerveModuleState(5, Rotation2d.fromDegrees(180)), angleB)
    };

    SwerveModuleState[] optimizedC = {
            new SwerveModuleState(5, Rotation2d.fromDegrees(180)),
            new SwerveModuleState(5, Rotation2d.fromDegrees(180)),
            new SwerveModuleState(5, Rotation2d.fromDegrees(180)),
            new SwerveModuleState(5, Rotation2d.fromDegrees(180))
    };

    var pose1 = m_odometry.updateWithTime(0.20, new Rotation2d(), optimizedB);
    var pose2 = m_odometry.updateWithTime(0.30, new Rotation2d(), optimizedC);

    assertAll(
            () -> assertEquals(0.0, pose1.getTranslation().x(), 0.01),
            () -> assertEquals(0, pose1.getTranslation().y(), 0.01),
            () -> assertEquals(0.0, pose1.getRotation().getDegrees(), 0.01)
    );

    assertAll(
            () -> assertEquals(-5.0 / 10 , pose2.getTranslation().x(), 0.01),
            () -> assertEquals(0, pose2.getTranslation().y(), 0.01),
            () -> assertEquals(0.0, pose2.getRotation().getDegrees(), 0.01)
    );

    assertAll(
            () -> assertEquals(optimizedB[0].speedInMetersPerSecond, -optimizedC[0].speedInMetersPerSecond, 0.01),
            () -> assertEquals(Angles.normalizeAngle(optimizedB[0].angle.getRadians()),
                    Angles.normalizeAngle(optimizedC[0].angle.getRadians() + Math.PI) , 0.01)
    );
  }
}