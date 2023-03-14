// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.estimation.CameraTargetRelation;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.NetworkTableConstants;
import frc.utils.MAXSwerveModule;
// import frc.utils.PhotonCameraWrapper;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private NetworkTableEntry m_xSpeedEntry = NetworkTableConstants.kDriveTable.getEntry("xSpeed");
  private NetworkTableEntry m_ySpeedEntry = NetworkTableConstants.kDriveTable.getEntry("ySpeed");
  private NetworkTableEntry m_rotSpeedEntry = NetworkTableConstants.kDriveTable.getEntry("rotSpeed");
  private NetworkTableEntry m_gyroHeadingEntry = NetworkTableConstants.kDriveTable.getEntry("gyroHeading");

  private Field2d m_field2d = new Field2d();
  Optional<EstimatedRobotPose> result;
  // public static PhotonCameraWrapper pcw = new PhotonCameraWrapper();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }, new Pose2d());

  public static boolean m_isInXForm = false;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Initialize the network table entries with the default values
    m_xSpeedEntry.setDefaultDouble(0.0);
    m_ySpeedEntry.setDefaultDouble(0.0);
    m_rotSpeedEntry.setDefaultDouble(0.0);
    m_gyroHeadingEntry.setDefaultDouble(0.0);
    SmartDashboard.putData(m_field2d);
  }

  int i = 0;

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    if (i > 20) {
      // updatePoseEstimate(i);
      i = 0;
    }
    i++;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param p_pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d p_pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        p_pose);
  }

  public void updatePoseEstimate(int i) {
    m_poseEstimator.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Optional<EstimatedRobotPose> result = pcw.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());
    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      m_poseEstimator.addVisionMeasurement(
          camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      m_field2d.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
      if (i > 20) {
        System.out.println(new CameraTargetRelation(new Pose3d(), camPose.estimatedPose).camToTargDistXY);
      }
    }

    m_field2d.getObject("Actual Pos").setPose(m_odometry.getPoseMeters());
    m_field2d.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    if (!m_isInXForm) {
      double xSpeedCommanded;
      double ySpeedCommanded;

      if (rateLimit) {
        // Convert XY to polar for rate limiting
        double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
        double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

        // Calculate the direction slew rate based on an estimate of the lateral
        // acceleration
        double directionSlewRate;
        if (m_currentTranslationMag != 0.0) {
          directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
        } else {
          directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
        }

        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - m_prevTime;
        double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
        if (angleDif < 0.45 * Math.PI) {
          m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
              directionSlewRate * elapsedTime);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        } else if (angleDif > 0.85 * Math.PI) {
          if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                // checking
            // keep currentTranslationDir unchanged
            m_currentTranslationMag = m_magLimiter.calculate(0.0);
          } else {
            m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
            m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
          }
        } else {
          m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
              directionSlewRate * elapsedTime);
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        m_prevTime = currentTime;

        xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
        ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
        m_currentRotation = m_rotLimiter.calculate(rot);

      } else {
        xSpeedCommanded = xSpeed;
        ySpeedCommanded = ySpeed;
        m_currentRotation = rot;
      }

      // Convert the commanded speeds into the correct units for the drivetrain
      double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
      double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
      double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;
      m_xSpeedEntry.setDouble(xSpeedDelivered);
      m_ySpeedEntry.setDouble(ySpeedDelivered);
      m_rotSpeedEntry.setDouble(rotDelivered);
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                  Rotation2d.fromDegrees(-m_gyro.getAngle()))
              : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_rearLeft.setDesiredState(swerveModuleStates[2]);
      m_rearRight.setDesiredState(swerveModuleStates[3]);
    }
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setXFormation() {
    m_isInXForm = true;
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void stopXFormation(){
    m_isInXForm = false;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param p_desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] p_desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        p_desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(p_desiredStates[0]);
    m_frontRight.setDesiredState(p_desiredStates[1]);
    m_rearLeft.setDesiredState(p_desiredStates[2]);
    m_rearRight.setDesiredState(p_desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /** Sets the Swerve Drive to certain angles */
  public void setSwerveDriveAngles(double backLeftAngle, double backRightAngle, double frontLeftAngle,
      double frontRightAngle) {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(frontLeftAngle)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(frontRightAngle)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(backLeftAngle)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(backRightAngle)));
  }

  /** Sets all the Swerve Drive Chassis Offsets */
  public void changeSwerveChassisOffsets(double kBackLeftChassisOffset, double kBackRightChassisOffset,
      double kFrontLeftChassisOffset, double kFrontRightChassisOffset) {
    m_frontLeft.setChassisOffset(kFrontLeftChassisOffset);
    m_frontRight.setChassisOffset(kFrontRightChassisOffset);
    m_rearLeft.setChassisOffset(kBackLeftChassisOffset);
    m_rearRight.setChassisOffset(kBackRightChassisOffset);
  }

  /**
   * Gets the MAXSwerveModules of the drive, for debugging purposes only.
   */
  public MAXSwerveModule[] getModules() {
    return new MAXSwerveModule[] { m_frontLeft, m_frontRight, m_rearLeft, m_rearRight };
  }

  /**
   * Gets the NavX gyro of the drive, for debugging purposes only.
   */
  public AHRS getGyro() {
    return m_gyro;
  }

  // Return the rest of the variables that are initialized in the beginning of the
  // class
  /** Returns the desired rotation of the robot */
  public double getDesiredRot() {
    return m_currentRotation;
  }

  /** Returns the desired translation direction of the robot */
  public double getDesiredTranslationDir() {
    return m_currentTranslationDir;
  }

  /** Returns the desired translation magnitude of the robot */
  public double getDesiredTranslationMag() {
    return m_currentTranslationMag;
  }
}