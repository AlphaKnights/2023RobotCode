// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.PneumaticConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.MAXSwerveModule;
import frc.robot.subsystems.PneumaticsSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetRobotStatusCommand extends InstantCommand {
  ArmSubsystem m_armSubsystem;
  DriveSubsystem m_robotDrive;
  ElevatorSubsystem m_elevatorSubsystem;
  PneumaticsSubsystem m_pneumaticsSubsystem;
  public GetRobotStatusCommand(ArmSubsystem p_armSubsystem, DriveSubsystem p_robotDrive, ElevatorSubsystem p_elevatorSubsystem, PneumaticsSubsystem p_pneumaticsSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = p_armSubsystem;
    m_robotDrive = p_robotDrive;
    m_elevatorSubsystem = p_elevatorSubsystem;
    m_pneumaticsSubsystem = p_pneumaticsSubsystem;
    addRequirements(m_armSubsystem, m_robotDrive, m_elevatorSubsystem, m_pneumaticsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Gets the status of the robot
    TalonFX armFalcon = m_armSubsystem.getArmFalcon();
    DigitalInput armRevLimit = m_armSubsystem.getLimitSwitch();
    TalonFX elevatorFalcon = m_elevatorSubsystem.getElevatorFalcon();
    DigitalInput elevatorRevLimit = m_elevatorSubsystem.getLimitSwitch();
    AHRS navX = m_robotDrive.getGyro();
    double heading = m_robotDrive.getHeading();
    double desiredRot = m_robotDrive.getDesiredRot();
    double getDesiredTranslationDir = m_robotDrive.getDesiredTranslationDir();
    double getDesiredTranslationMag = m_robotDrive.getDesiredTranslationMag();
    MAXSwerveModule[] maxSwerveModules = m_robotDrive.getModules();
    Pose2d pose2d = m_robotDrive.getPose();
    double turnRate = m_robotDrive.getTurnRate();
    DoubleSolenoid claw = m_pneumaticsSubsystem.getClawPiston();
    DoubleSolenoid wrist = m_pneumaticsSubsystem.getWristPiston();
    PneumaticHub pneumaticHub = m_pneumaticsSubsystem.getPneumaticHub();

    //Prints the status of the robot subsystems and their components to the console
    System.out.println("Arm Falcon: ");
    System.out.println("  Bus voltage: "+armFalcon.getBusVoltage());
    System.out.println("  ID: "+armFalcon.getDeviceID());
    System.out.println("  Firmware version: "+armFalcon.getFirmwareVersion());
    System.out.println("  Output %:"+armFalcon.getMotorOutputPercent());
    System.out.println("  Output voltage: "+armFalcon.getMotorOutputVoltage());
    System.out.println("  Encoder Pos: "+armFalcon.getSelectedSensorPosition());
    System.out.println("  Encoder Vel: "+armFalcon.getSelectedSensorVelocity());
    System.out.println("  Stator Current: "+armFalcon.getStatorCurrent());
    System.out.println("  Supply Current: "+armFalcon.getSupplyCurrent());
    System.out.println("  Temperature: "+armFalcon.getTemperature());
    System.out.println("  Control Mode: "+armFalcon.getControlMode());
    System.out.println("");
    System.out.println("  Limit Switch: "+armRevLimit.get());
    System.out.println("");
    System.out.println("Elevator Falcon: ");
    System.out.println("  Bus voltage: "+elevatorFalcon.getBusVoltage());
    System.out.println("  ID: "+elevatorFalcon.getDeviceID());
    System.out.println("  Firmware version: "+elevatorFalcon.getFirmwareVersion());
    System.out.println("  Output %:"+elevatorFalcon.getMotorOutputPercent());
    System.out.println("  Output voltage: "+elevatorFalcon.getMotorOutputVoltage());
    System.out.println("  Encoder Pos: "+elevatorFalcon.getSelectedSensorPosition());
    System.out.println("  Encoder Vel: "+elevatorFalcon.getSelectedSensorVelocity());
    System.out.println("  Stator Current: "+elevatorFalcon.getStatorCurrent());
    System.out.println("  Supply Current: "+elevatorFalcon.getSupplyCurrent());
    System.out.println("  Temperature: "+elevatorFalcon.getTemperature());
    System.out.println("  Control Mode: "+elevatorFalcon.getControlMode());
    System.out.println("");
    System.out.println("  Limit Switch: "+elevatorRevLimit.get());
    System.out.println("");
    System.out.println("Gyro: ");
    System.out.println("  Altitude: "+navX.getAltitude());
    System.out.println("  Angle: "+navX.getAngle());
    System.out.println("  Angle Adjustment: "+navX.getAngleAdjustment());
    System.out.println("  Barometric Pressure: "+navX.getBarometricPressure());
    System.out.println("  Compass Heading: "+navX.getCompassHeading());
    System.out.println("  Displacement X: "+navX.getDisplacementX());
    System.out.println("  Displacement Y: "+navX.getDisplacementY());
    System.out.println("  Displacement Z: "+navX.getDisplacementZ());
    System.out.println("  Firmware Version: "+navX.getFirmwareVersion());
    System.out.println("  Fused Heading: "+navX.getFusedHeading());
    System.out.println("  Gyro Full Range DPS: "+navX.getGyroFullScaleRangeDPS());
    System.out.println("  Last Sensor Timestamp: "+navX.getLastSensorTimestamp());
    System.out.println("  Pitch: "+navX.getPitch());
    System.out.println("  Pressure: "+navX.getPressure());
    System.out.println("  Quaternion W: "+navX.getQuaternionW());
    System.out.println("  Quaternion X: "+navX.getQuaternionX());
    System.out.println("  Quaternion Y: "+navX.getQuaternionY());
    System.out.println("  Quaternion Z: "+navX.getQuaternionZ());
    System.out.println("  Raw Accel X: "+navX.getRawAccelX());
    System.out.println("  Raw Accel Y: "+navX.getRawAccelY());
    System.out.println("  Raw Accel Z: "+navX.getRawAccelZ());
    System.out.println("  Raw Mag X: "+navX.getRawMagX());
    System.out.println("  Raw Mag Y: "+navX.getRawMagY());
    System.out.println("  Raw Mag Z: "+navX.getRawMagZ());
    System.out.println("  Update Rate: "+navX.getRequestedUpdateRate());
    System.out.println("  Roll: "+navX.getRoll());
    System.out.println("  Temp C: "+navX.getTempC());
    System.out.println("  Update Count: "+navX.getUpdateCount());
    System.out.println("  Velocity X: "+navX.getVelocityX());
    System.out.println("  Velocity Y: "+navX.getVelocityY());
    System.out.println("  Velocity Z: "+navX.getVelocityZ());
    System.out.println("  World Liner Accel X: "+navX.getWorldLinearAccelX());
    System.out.println("  World Liner Accel Y: "+navX.getWorldLinearAccelY());
    System.out.println("  World Liner Accel Z: "+navX.getWorldLinearAccelZ());
    System.out.println("  Board Yaw: "+navX.getBoardYawAxis());
    System.out.println("  Yaw: "+navX.getYaw());
    System.out.println("  Rotation 2d: "+navX.getRotation2d());
    System.out.println("    Deg: "+navX.getRotation2d().getDegrees());
    System.out.println("    Rotations: "+navX.getRotation2d().getRotations());
    System.out.println("    Radians: "+navX.getRotation2d().getRadians());
    System.out.println("");
    System.out.println("Swerve: ");
    System.out.println("  Front Left: ");
    System.out.println("    Position: "+maxSwerveModules[0].getPosition());
    System.out.println("      Distance: "+maxSwerveModules[0].getPosition().distanceMeters);
    System.out.println("      Angle: "+maxSwerveModules[0].getPosition().angle.getDegrees());
    System.out.println("    State: "+maxSwerveModules[0].getState());
    System.out.println("      Speed: "+maxSwerveModules[0].getState().speedMetersPerSecond);
    System.out.println("      Angle: "+maxSwerveModules[0].getState().angle);
    System.out.println("  Front Right: ");
    System.out.println("    Position: "+maxSwerveModules[1].getPosition());
    System.out.println("      Distance: "+maxSwerveModules[1].getPosition().distanceMeters);
    System.out.println("      Angle: "+maxSwerveModules[1].getPosition().angle.getDegrees());
    System.out.println("    State: "+maxSwerveModules[1].getState());
    System.out.println("      Speed: "+maxSwerveModules[1].getState().speedMetersPerSecond);
    System.out.println("      Angle: "+maxSwerveModules[1].getState().angle);
    System.out.println("  Back Left: ");
    System.out.println("    Position: "+maxSwerveModules[2].getPosition());
    System.out.println("      Distance: "+maxSwerveModules[2].getPosition().distanceMeters);
    System.out.println("      Angle: "+maxSwerveModules[2].getPosition().angle.getDegrees());
    System.out.println("    State: "+maxSwerveModules[2].getState());
    System.out.println("      Speed: "+maxSwerveModules[2].getState().speedMetersPerSecond);
    System.out.println("      Angle: "+maxSwerveModules[2].getState().angle);
    System.out.println("  Back Right: ");
    System.out.println("    Position: "+maxSwerveModules[3].getPosition());
    System.out.println("      Distance: "+maxSwerveModules[3].getPosition().distanceMeters);
    System.out.println("      Angle: "+maxSwerveModules[3].getPosition().angle.getDegrees());
    System.out.println("    State: "+maxSwerveModules[3].getState());
    System.out.println("      Speed: "+maxSwerveModules[3].getState().speedMetersPerSecond);
    System.out.println("      Angle: "+maxSwerveModules[3].getState().angle);
    System.out.println("");
    System.out.println("Other Swerve: ");
    System.out.println("  Heading: "+heading);
    System.out.println("  desiredRot: "+desiredRot);
    System.out.println("  Desired Translation Dir: "+getDesiredTranslationDir);
    System.out.println("  Desired Translation Mag: "+getDesiredTranslationMag);
    System.out.println("  Turn Rate: "+turnRate);
    System.out.println("  Position: "+pose2d);
    System.out.println("    X: "+pose2d.getX());
    System.out.println("    Y: "+pose2d.getY());
    System.out.println("    Rotation: "+pose2d.getRotation());
    System.out.println("      Deg: "+pose2d.getRotation().getDegrees());
    System.out.println("      Rotations: "+pose2d.getRotation().getRotations());
    System.out.println("      Radians: "+pose2d.getRotation().getRadians());
    System.out.println("    Translation:"+pose2d.getTranslation());
    System.out.println("      X: "+pose2d.getTranslation().getX());
    System.out.println("      Y: "+pose2d.getTranslation().getY());
    System.out.println("      Norm: "+pose2d.getTranslation().getNorm());
    System.out.println("      Angle: "+pose2d.getTranslation().getAngle());
    System.out.println("Pneumatics: ");
    System.out.println("  Claw: "+claw.get());
    System.out.println("  Wrist: "+wrist.get());
    System.out.println("    Pneumatic Hub: ");
    System.out.println("      5v Reg Voltage: "+pneumaticHub.get5VRegulatedVoltage());
    System.out.println("      Compressor Current: "+pneumaticHub.getCompressorCurrent());
    System.out.println("      Input Current: "+pneumaticHub.getInputVoltage());
    System.out.println("      Module Number: "+pneumaticHub.getModuleNumber());
    System.out.println("      Module Number: "+pneumaticHub.getPressure(PneumaticConstants.kAnalogSensorPort));
    System.out.println("      Solonoid Disabled List: "+pneumaticHub.getSolenoidDisabledList());
    System.out.println("      Solonoids: "+pneumaticHub.getSolenoids());
    System.out.println("      Solonoids Total Current: "+pneumaticHub.getSolenoidsTotalCurrent());
    System.out.println("      Solonoids Voltage: "+pneumaticHub.getSolenoidsVoltage());
    System.out.println("      Compressor: "+pneumaticHub.getCompressor());
    System.out.println("      Compressor Config Type: "+pneumaticHub.getCompressorConfigType());
    System.out.println("      Faults: "+pneumaticHub.getFaults());
    System.out.println("      Sticky Faults: "+pneumaticHub.getStickyFaults());
    System.out.println("      Pressure Switch: "+pneumaticHub.getPressureSwitch());
    System.out.println("      Version: "+pneumaticHub.getVersion());
  }
}
