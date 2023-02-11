// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final TalonFX m_drivingFalcon;
  private final CANSparkMax m_turningSparkMax;

  private final AbsoluteEncoder m_turningEncoder;

  private final SparkMaxPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingFalcon = new TalonFX(drivingCANId);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    //Falcon 500 Configuration with PID
    m_drivingFalcon.setNeutralMode(NeutralMode.Brake);
    m_drivingFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,Constants.PIDConstants.pidIdx, Constants.PIDConstants.kTimeoutMs);//, 
    m_drivingFalcon.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.IntegratedSensor, Constants.PIDConstants.kTimeoutMs);
    m_drivingFalcon.configSelectedFeedbackCoefficient(1, Constants.PIDConstants.pidIdx, Constants.PIDConstants.kTimeoutMs);
    m_drivingFalcon.configNeutralDeadband(Constants.PIDConstants.kNeutralDeadband, Constants.PIDConstants.kTimeoutMs);
    m_drivingFalcon.configMotionAcceleration(Constants.PIDConstants.sensorUnitsPer100msPerSec, Constants.PIDConstants.kTimeoutMs);

    m_drivingFalcon.config_kP(Constants.PIDConstants.kSlot, Constants.PIDConstants.kP, Constants.PIDConstants.kTimeoutMs);
		m_drivingFalcon.config_kI(Constants.PIDConstants.kSlot, Constants.PIDConstants.kI, Constants.PIDConstants.kTimeoutMs);
		m_drivingFalcon.config_kD(Constants.PIDConstants.kSlot, Constants.PIDConstants.kD, Constants.PIDConstants.kTimeoutMs);
		m_drivingFalcon.config_kF(Constants.PIDConstants.kSlot, Constants.PIDConstants.kF, Constants.PIDConstants.kTimeoutMs);
		m_drivingFalcon.config_IntegralZone(Constants.PIDConstants.kSlot, Constants.PIDConstants.kIzone, Constants.PIDConstants.kTimeoutMs);
		m_drivingFalcon.configClosedLoopPeakOutput(Constants.PIDConstants.kSlot, Constants.PIDConstants.kPeakOutput, Constants.PIDConstants.kTimeoutMs);

    
		int closedLoopTimeMs = 1;
		m_drivingFalcon.configClosedLoopPeriod(Constants.PIDConstants.kSlot, closedLoopTimeMs, Constants.PIDConstants.kTimeoutMs);
		m_drivingFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
    m_drivingFalcon.setSelectedSensorPosition(0);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the turning motor
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    // m_drivingEncoder.setPosition(0);
    // m_drivingFalcon.getSensorCollection().setIntegratedSensorPosition(0, Constants.PIDConstants.kTimeoutMs);
    // m_drivingFalcon
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingFalcon.getSelectedSensorVelocity(0),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingFalcon.getSelectedSensorPosition(0),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, int canId) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingFalcon.set(TalonFXControlMode.PercentOutput, optimizedDesiredState.speedMetersPerSecond/9.6);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingFalcon.getSensorCollection().setIntegratedSensorPosition(0, Constants.PIDConstants.kTimeoutMs);
  }
}
