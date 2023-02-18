// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OIConstants {
    //PSP Controller
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kNavXZeroButton = 1;
    public static final int kResetEncodersButton = 2;
    public static final int kHoldPositionButton = 3;
    //Left Flight Stick
    public static final int kLeftJoystickControllerPort = 0; // Default port for the left flight joystick
    public static final double kLeftJoystickDeadband = 0.05; // Deadband for the left flight joystick
    //Right Flight Stick
    public static final int kRightJoystickControllerPort = 1; // Default port for the right flight joystick
    public static final double kRightJoystickDeadband = 0.05; // Deadband for the right flight joystick
    public static final int toggleCompressorButton = 2; // Thumb Button
    public static final int clawShutButton = 1; // Trigger Button
    public static final int clawHalfShutButton = 3; // Button 3
    public static final int clawOpenButton = 4; // Button 4
    public static final int clawOffButton = 6; // Button 4
  }

  public static class DriveConstants {//TODO: Convert class to final once constants are tuned
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;//TODO: get velocity working with the falcon 500
    public static final double kMaxAngularSpeed = .25 * Math.PI; // radians per second

    //slew containts to add motion curve
    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; 
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.125);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(32);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in degrees 
    
    //TODO: Temporary comment out till constants are tuned, once tuned insert values and uncomment below:

    // public static final double kFrontLeftChassisAngularOffsetDegrees = 0;
    // public static final double kFrontRightChassisAngularOffsetDegrees = 0;
    // public static final double kBackLeftChassisAngularOffsetDegrees = -90;
    // public static final double kBackRightChassisAngularOffsetDegrees = -30;

    // // Angular offsets of the modules relative to the chassis in radians
    // public static final double kFrontLeftChassisAngularOffset = kFrontLeftChassisAngularOffsetDegrees*(Math.PI/180);
    // public static final double kFrontRightChassisAngularOffset = kFrontRightChassisAngularOffsetDegrees*(Math.PI/180);
    // public static final double kBackLeftChassisAngularOffset = kBackLeftChassisAngularOffsetDegrees*(Math.PI/180);
    // public static final double kBackRightChassisAngularOffset = kBackRightChassisAngularOffsetDegrees*(Math.PI/180);

    public static double kFrontLeftChassisAngularOffsetDegrees = 0;
    public static double kFrontRightChassisAngularOffsetDegrees = 0;
    public static double kBackLeftChassisAngularOffsetDegrees = -90;
    public static double kBackRightChassisAngularOffsetDegrees = -30;

    // Angular offsets of the modules relative to the chassis in radians
    public static double kFrontLeftChassisAngularOffset = kFrontLeftChassisAngularOffsetDegrees*(Math.PI/180);
    public static double kFrontRightChassisAngularOffset = kFrontRightChassisAngularOffsetDegrees*(Math.PI/180);
    public static double kBackLeftChassisAngularOffset = kBackLeftChassisAngularOffsetDegrees*(Math.PI/180);
    public static double kBackRightChassisAngularOffset = kBackRightChassisAngularOffsetDegrees*(Math.PI/180);

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 5;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 11;
    public static final int kFrontRightTurningCanId = 9;
    public static final int kRearRightTurningCanId = 12;

    public static final boolean kGyroReversed = true;
  }

  public static final class ElevatorConstants{
    public static final double kSensorCountPerRevolution = 2048;
    public static final double kMaxFowardVerticalRotation = 5;// Up
    public static final double kMaxReverseVerticalRotation = 0; //Down, set to 0 as it starts in the fully down state so it starts in pos 0
    public static final double kFowardVerticalCount = kMaxFowardVerticalRotation*kSensorCountPerRevolution;
    public static final double kReverseVerticalCount = kMaxReverseVerticalRotation*kSensorCountPerRevolution;
    public static final int kElevatorFalconID = 0;
    public static final int kLimitSwitchPort = 0;
  }

  public static final class ArmConstants{
    public static final double kSensorCountPerRevolution = 2048;
    public static final double kMaxFowardRotation = 5;// Out
    public static final double kMaxReverseRotation = 0; //In, starts fully in so it starts in pos 0
    public static final double kFowardRotationCount = kMaxFowardRotation*kSensorCountPerRevolution;
    public static final double kReverseRotationCount = kMaxReverseRotation*kSensorCountPerRevolution;
    public static final int kArmFalconID = 0;
    public static final int kLimitSwitchPort = 0;
  }

  public static final class ClawConstants{
    public static final int kRevPneumaticPort = 2;
    public static final int kPiston1FwdPort = 0;
    public static final int kPiston1RevPort = 1;
    public static final int kPiston2FwdPort = 2;
    public static final int kPiston2RevPort = 3;
    public static final double kClawMinPressure = 40;
    public static final double kClawMaxPressure = 50;
  }

  

  public static enum PistonState {
    OPEN, HALF, CLOSED, OFF
  }

  public static final class PIDConstants{
    public static final int pidIdx = 0;
    public static final int kTimeoutMs = 100;
    public static final int kNeutralDeadband = 1;
    public static final int sensorUnitsPer100msPerSec = 2000;
    public static final int kSlot = 0;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kF = 0.0;
    public static final double kIzone = 0.0;
    public static final double kPeakOutput = 0.0;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = DriveMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = Math.PI/180; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = .5;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class DriveMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
