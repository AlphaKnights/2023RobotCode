// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.NetworkTableConstants;

public class ElevatorSubsystem extends SubsystemBase {
  //Config the falcon and the limit switches
  TalonFX elevatorFalcon = new TalonFX(ElevatorConstants.kElevatorFalconID);
  TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
  DigitalInput reverseLimit = new DigitalInput(ElevatorConstants.kReverseLimitSwitchPort);
  DigitalInput forwardLimit = new DigitalInput(ElevatorConstants.kForwardLimitSwitchPort);
  PIDController elevatorPID = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
  private double m_maxElevatorVerticalPosition = ElevatorConstants.kDefaultMaxFowardVerticalRotation;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorConfig.forwardSoftLimitEnable = true;
    elevatorConfig.reverseSoftLimitEnable = false;
    elevatorConfig.forwardSoftLimitThreshold = m_maxElevatorVerticalPosition;
    elevatorConfig.reverseSoftLimitThreshold = ElevatorConstants.kMaxReverseVerticalRotationCount;

    elevatorConfig.slot0.kP = ElevatorConstants.kP;
    elevatorConfig.slot0.kI = ElevatorConstants.kI;
    elevatorConfig.slot0.kD = ElevatorConstants.kD;
    elevatorConfig.slot0.kF = ElevatorConstants.kF;
    elevatorConfig.slot0.integralZone = ElevatorConstants.kIZone;
    elevatorConfig.slot0.closedLoopPeakOutput = ElevatorConstants.kPeakOutput;
    // elevatorConfig.slot0.allowableClosedloopError = ElevatorConstants.kPositionTolerance;

    elevatorFalcon.configAllSettings(elevatorConfig);
    elevatorFalcon.setNeutralMode(NeutralMode.Brake);
    elevatorFalcon.setSelectedSensorPosition(ElevatorConstants.kMaxReverseVerticalRotationCount);
    // elevatorFalcon.config
  }
  int i = 1;
  @Override
  public void periodic() {
    if(NetworkTableConstants.DEBUG){
      if(i>=10){
        System.out.println(elevatorFalcon.getSelectedSensorPosition());
        i = 1;
      }
      else{
        i++;
      }
    }
    // System.out.println(reverseLimit.get());

    // This method will be called once per scheduler run
  }

  /**
   * 
   * @param p_power the power to set the elevator to, between -1 and 1. It will refuse to move in the reverse direction if the limit switch is pressed.
   */
  public void setPower(double p_power) {
    if (reverseLimit.get()) {
      if(p_power<0){
        elevatorFalcon.set(ControlMode.PercentOutput, 0);
      }
      else{
        elevatorFalcon.set(ControlMode.PercentOutput, p_power);
      }
      elevatorFalcon.setSelectedSensorPosition(ElevatorConstants.kMaxReverseVerticalRotationCount);
    } else if (!forwardLimit.get()){//Normally Open, so if it's not pressed, it's true
      if(p_power>0){
        elevatorFalcon.set(ControlMode.PercentOutput, 0);
      }
      else{
        elevatorFalcon.set(ControlMode.PercentOutput, p_power);
      }
      // elevatorFalcon.setSelectedSensorPosition(ElevatorConstants.kFowardVerticalCount);
    } else {
     elevatorFalcon.set(ControlMode.PercentOutput, p_power);
    }
  }

  /**
   * 
   * @param p_position the position to move the elevator to, in encoder counts.
   */
  public void goToPosition(double p_position) {
    // if(reverseLimit.get()){
    //   elevatorFalcon.setSelectedSensorPosition(ElevatorConstants.kReverseVerticalCount);
    // }
    // if(elevatorFalcon.getSelectedSensorPosition()<p_position-10){
    //   // elevatorFalcon.set(ControlMode.PercentOutput, .5);
    //   setPower(.5);
    // }
    // else if(elevatorFalcon.getSelectedSensorPosition()>p_position+10){
    //   // elevatorFalcon.set(ControlMode.PercentOutput, .5);
    //   setPower(-.5);
    // }
    // elevatorFalcon.set(ControlMode.MotionMagic, p_position);
    setPower(MathUtil.clamp(elevatorPID.calculate(elevatorFalcon.getSelectedSensorPosition(), p_position*ElevatorConstants.kF),-1*ElevatorConstants.kPeakOutput,ElevatorConstants.kPeakOutput));
    // elevatorFalcon.set(ControlMode.Position, p_position);
  }

  public boolean homeElevator(){
    if(homeElevatorBottom()){
      return homeElevatorTop();
    }
    else{
      return homeElevatorBottom();
    }
  }

  public boolean homeElevatorBottom(){
    if(reverseLimit.get()){
      elevatorFalcon.setSelectedSensorPosition(ElevatorConstants.kMaxReverseVerticalRotationCount);
      setPower(0);
      return true;
    }
    else{
      setPower(-.5);
      return false;
    }
  }

  public boolean homeElevatorTop(){
    if(forwardLimit.get()){
      elevatorFalcon.configForwardSoftLimitThreshold(elevatorFalcon.getSelectedSensorPosition());
      setPower(0);
      return true;
    }
    else{
      setPower(.5);
      return false;
    }
  }
  /**
   * 
   * @param p_position the position to move the elevator to, in encoder counts.
   */
  public double getPosition() {
    return elevatorFalcon.getSelectedSensorPosition();
  }
  
  /**
   * Gets the elevator Falcon object, for debugging use only.
   */
  public TalonFX getElevatorFalcon() {
    return elevatorFalcon;
  }

  /**
   * Gets the elevator limit switch object, for debugging use only.
   */
  public DigitalInput getLimitSwitch() {
    return reverseLimit;
  }
}
