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
import edu.wpi.first.networktables.NetworkTableEntry;
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
  NetworkTableEntry armPEntry = NetworkTableConstants.kArmTable.getEntry("P");
  NetworkTableEntry armIEntry = NetworkTableConstants.kArmTable.getEntry("I");
  NetworkTableEntry armDEntry = NetworkTableConstants.kArmTable.getEntry("D");
  NetworkTableEntry armFwdLimit = NetworkTableConstants.kArmTable.getEntry("FwdLimit");
  NetworkTableEntry armRvsLimit = NetworkTableConstants.kArmTable.getEntry("RvsLimit");
  NetworkTableEntry armPosition = NetworkTableConstants.kArmTable.getEntry("Position");
  NetworkTableEntry armPower = NetworkTableConstants.kArmTable.getEntry("Power");
  NetworkTableEntry ingoreArmFwdLimit = NetworkTableConstants.kArmTable.getEntry("IngoreFwdLimit");

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armPEntry.setDefaultDouble(ArmConstants.kP);
    armIEntry.setDefaultDouble(ArmConstants.kI);
    armDEntry.setDefaultDouble(ArmConstants.kD);
    armFwdLimit.setDefaultBoolean(false);
    armRvsLimit.setDefaultBoolean(false);
    armPosition.setDefaultDouble(ArmConstants.kReverseRotationCount);
    armPower.setDefaultDouble(0);
    ingoreArmFwdLimit.setDefaultBoolean(false);
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
  int j = 1;
  @Override
  public void periodic() {
    armFwdLimit.setBoolean(ArmFalcon.getStatorCurrent()>ArmConstants.kStallCurrent);
    armRvsLimit.setBoolean(reverseLimit.get());
    armPosition.setDouble(ArmFalcon.getSelectedSensorPosition()/ArmConstants.kSensorCountPerRevolution);
    armPower.setDouble(ArmFalcon.getStatorCurrent());
    // if(NetworkTableConstants.DEBUG){
      // if(i>=25){
      //   i = 1;
      // }
      // else{
      //   i++;
      // }
    // }

  }

  /**
   * 
   * @param p_power the power to set the Arm to, between -1 and 1. It will refuse to move in the reverse direction if the limit switch is pressed.
   */
  public void setPower(double p_power) {
    if (reverseLimit.get()) {
      // System.out.println("Rvs Limit");
      if(p_power>0){
        // System.out.println("limit two");
        ArmFalcon.set(ControlMode.PercentOutput, 0);
      }
      else{
        ArmFalcon.set(ControlMode.PercentOutput, p_power);
      }
      ArmFalcon.setSelectedSensorPosition(ArmConstants.kReverseRotationCount);
    } else if (ArmFalcon.getStatorCurrent()>ArmConstants.kStallCurrent&&!ingoreArmFwdLimit.getBoolean(false)){
      if(p_power<0){
        ArmFalcon.set(ControlMode.PercentOutput, 0);
        m_maxArmPosition = ArmFalcon.getSelectedSensorPosition();
        
      }
      else{
        ArmFalcon.set(ControlMode.PercentOutput, p_power);
      }
    } else {
     ArmFalcon.set(ControlMode.PercentOutput, p_power);
    }
  }

  /**
   * 
   * @param p_position the position to move the Arm to, in encoder counts.
   */
  public void goToPosition(double p_position) {
    ArmPID.setP(armPEntry.getDouble(ArmConstants.kP));
    ArmPID.setI(armIEntry.getDouble(ArmConstants.kI));
    ArmPID.setD(armDEntry.getDouble(ArmConstants.kD));
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
