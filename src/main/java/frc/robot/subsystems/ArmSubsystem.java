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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.NetworkTableConstants;

public class ArmSubsystem extends SubsystemBase {
  //Config the falcon and the limit switches
  TalonFX ArmFalcon = new TalonFX(ArmConstants.kArmFalconID);
  TalonFXConfiguration ArmConfig = new TalonFXConfiguration();
  DigitalInput reverseLimit = new DigitalInput(ArmConstants.kLimitSwitchPort);
  // DigitalInput forwardLimit = new DigitalInput(ArmConstants.kForwardLimitSwitchPort);
  PIDController ArmPID = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
  private double m_maxArmPosition = ArmConstants.kDefaultMaxFowardRotationCount;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    ArmConfig.forwardSoftLimitEnable = true;
    ArmConfig.reverseSoftLimitEnable = false;
    ArmConfig.forwardSoftLimitThreshold = m_maxArmPosition;
    ArmConfig.reverseSoftLimitThreshold = ArmConstants.kReverseRotationCount;

    ArmConfig.slot0.kP = ArmConstants.kP;
    ArmConfig.slot0.kI = ArmConstants.kI;
    ArmConfig.slot0.kD = ArmConstants.kD;
    ArmConfig.slot0.kF = ArmConstants.kF;
    ArmConfig.slot0.integralZone = ArmConstants.kIZone;
    ArmConfig.slot0.closedLoopPeakOutput = ArmConstants.kPeakOutput;
    // ArmConfig.slot0.allowableClosedloopError = ArmConstants.kPositionTolerance;

    ArmFalcon.configAllSettings(ArmConfig);
    ArmFalcon.setNeutralMode(NeutralMode.Brake);
    ArmFalcon.setSelectedSensorPosition(ArmConstants.kReverseRotationCount);
    // ArmFalcon.config
  }
  int i = 1;
  @Override
  public void periodic() {
    if(NetworkTableConstants.DEBUG){
      if(i>=10){
        System.out.println(ArmFalcon.getSelectedSensorPosition());
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
   * @param p_power the power to set the Arm to, between -1 and 1. It will refuse to move in the reverse direction if the limit switch is pressed.
   */
  public void setPower(double p_power) {
    if (reverseLimit.get()) {
      // System.out.println("Rvs Limit");
      if(p_power<0){
        ArmFalcon.set(ControlMode.PercentOutput, 0);
      }
      else{
        ArmFalcon.set(ControlMode.PercentOutput, p_power);
      }
      ArmFalcon.setSelectedSensorPosition(ArmConstants.kReverseRotationCount);
    } else if (ArmFalcon.getStatorCurrent()>ArmConstants.kStallCurrent){
      // System.out.println("Fwd Limit");
      if(p_power>0){
        ArmFalcon.set(ControlMode.PercentOutput, 0);
      }
      else{
        ArmFalcon.set(ControlMode.PercentOutput, p_power);
      }
      m_maxArmPosition = ArmFalcon.getSelectedSensorPosition();
      ArmFalcon.configForwardSoftLimitThreshold(m_maxArmPosition);
      // ArmFalcon.setSelectedSensorPosition(ArmConstants.kFowardRotationCount);
    } else {
     ArmFalcon.set(ControlMode.PercentOutput, p_power);
    }
  }

  /**
   * 
   * @param p_position the position to move the Arm to, in encoder counts.
   */
  public void goToPosition(double p_position) {
    // if(reverseLimit.get()){
    //   ArmFalcon.setSelectedSensorPosition(ArmConstants.kReverseRotationCount);
    // }
    // if(ArmFalcon.getSelectedSensorPosition()<p_position-10){
    //   // ArmFalcon.set(ControlMode.PercentOutput, .5);
    //   setPower(.5);
    // }
    // else if(ArmFalcon.getSelectedSensorPosition()>p_position+10){
    //   // ArmFalcon.set(ControlMode.PercentOutput, .5);
    //   setPower(-.5);
    // }
    // ArmFalcon.set(ControlMode.MotionMagic, p_position);
    setPower(MathUtil.clamp(ArmPID.calculate(ArmFalcon.getSelectedSensorPosition(), p_position*ArmConstants.kF),-1*ArmConstants.kPeakOutput,ArmConstants.kPeakOutput));
    // ArmFalcon.set(ControlMode.Position, p_position);
  }

  public boolean homeArm(){
    if(homeArmBottom()){
      return homeArmTop();
    }
    else{
      return homeArmBottom();
    }
  }

  public boolean homeArmBottom(){
    if(reverseLimit.get()){
      ArmFalcon.setSelectedSensorPosition(ArmConstants.kReverseRotationCount);
      setPower(0);
      return true;
    }
    else{
      setPower(-.5);
      return false;
    }
  }

  public boolean homeArmTop(){
    if(ArmFalcon.getStatorCurrent()>ArmConstants.kStallCurrent){
      ArmFalcon.configForwardSoftLimitThreshold(ArmFalcon.getSelectedSensorPosition());
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
   * @param p_position the position to move the Arm to, in encoder counts.
   */
  public double getPosition() {
    return ArmFalcon.getSelectedSensorPosition();
  }
  
  /**
   * Gets the Arm Falcon object, for debugging use only.
   */
  public TalonFX getArmFalcon() {
    return ArmFalcon;
  }

  /**
   * Gets the Arm limit switch object, for debugging use only.
   */
  public DigitalInput getLimitSwitch() {
    return reverseLimit;
  }
}
