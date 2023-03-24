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
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.NetworkTableConstants;
// import frc.robot.subsystems.ElevatorSubsystem;

public class ArmSubsystem extends SubsystemBase { // add either a public static double elvPower or a parameter for SetPower then go to line with -----
  private double elvPos;
  private double extPos;  //Config the falcon and the limit switches
  TalonFX ArmFalcon = new TalonFX(ArmConstants.kArmFalconID);
  TalonFXConfiguration ArmConfig = new TalonFXConfiguration();
  DigitalInput reverseLimit = new DigitalInput(ArmConstants.kLimitSwitchPort);
  PIDController ArmPID = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
  private double m_maxArmPosition = ArmConstants.kDefaultMaxFowardRotationCount;
  NetworkTableEntry armPEntry = NetworkTableConstants.kArmTable.getEntry("P");
  NetworkTableEntry armIEntry = NetworkTableConstants.kArmTable.getEntry("I");
  NetworkTableEntry armDEntry = NetworkTableConstants.kArmTable.getEntry("D");
  NetworkTableEntry armFwdLimitEntry = NetworkTableConstants.kArmTable.getEntry("FwdLimit");
  NetworkTableEntry armRvsLimitEntry = NetworkTableConstants.kArmTable.getEntry("RvsLimit");
  NetworkTableEntry armPositionEntry = NetworkTableConstants.kArmTable.getEntry("Position");
  NetworkTableEntry armPowerEntry = NetworkTableConstants.kArmTable.getEntry("Power");
  NetworkTableEntry ingoreArmFwdLimitEntry = NetworkTableConstants.kArmTable.getEntry("IngoreFwdLimit");
  NetworkTableEntry powerLimitEntry = NetworkTableConstants.kArmTable.getEntry("PowerLimit");
  NetworkTableEntry ignoreLimitEntry = NetworkTableConstants.kArmTable.getEntry("IgnoreLimit");
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armPEntry.setDefaultDouble(ArmConstants.kP);
    armIEntry.setDefaultDouble(ArmConstants.kI);
    armDEntry.setDefaultDouble(ArmConstants.kD);
    armFwdLimitEntry.setDefaultBoolean(false);
    armRvsLimitEntry.setDefaultBoolean(false);
    armPositionEntry.setDefaultDouble(ArmConstants.kReverseRotationCount);
    armPowerEntry.setDefaultDouble(0);
    ingoreArmFwdLimitEntry.setDefaultBoolean(false);
    powerLimitEntry.setDefaultDouble(ArmConstants.kStallCurrent);

    ArmConfig.forwardSoftLimitEnable = false;
    ArmConfig.reverseSoftLimitEnable = false;
    ArmConfig.forwardSoftLimitThreshold = m_maxArmPosition;
    ArmConfig.reverseSoftLimitThreshold = ArmConstants.kReverseRotationCount;
    ArmConfig.slot0.kP = ArmConstants.kP;
    ArmConfig.slot0.kI = ArmConstants.kI;
    ArmConfig.slot0.kD = ArmConstants.kD;
    ArmConfig.slot0.kF = ArmConstants.kF;
    ArmConfig.slot0.integralZone = ArmConstants.kIZone;
    ArmConfig.slot0.closedLoopPeakOutput = ArmConstants.kPeakOutput;

    ArmFalcon.configAllSettings(ArmConfig);
    ArmFalcon.setNeutralMode(NeutralMode.Brake);
    ArmFalcon.setSelectedSensorPosition(ArmConstants.kReverseRotationCount);
  }
  
  @Override
  public void periodic() {
  }

  /**
   * power to extension = elevator position * power to elevation / sqrt(elevator position^2+maximum^2)
   * setPower(elvPos*elvcontroller/sqrt(elvpos^2+ArmConstants.maxHorizontal extension^2));
   * @param p_power the power to set the Arm to, between -1 and 1. It will refuse to move in the reverse direction if the limit switch is pressed.
   */
  public void setPower(double p_power, ElevatorSubsystem p_ElevatorSubsystem) {
    elvPos = p_ElevatorSubsystem.getPosition()/ArmConstants.kEncoderTicksPerRotation*1.43*Math.PI;
    extPos = -ArmFalcon.getSelectedSensorPosition()/ArmConstants.kEncoderTicksPerRotation*1.43*Math.PI+41;
    System.out.println(ArmFalcon.getSelectedSensorPosition() + "  <ext encoder    elv encoder>" + elvPos + "    ignore>    " + p_power + "   double check: " + -ArmFalcon.getSelectedSensorPosition()/ArmConstants.kEncoderTicksPerRotation*1.43*Math.PI);
    armRvsLimitEntry.setBoolean(reverseLimit.get());
    armFwdLimitEntry.setBoolean(ArmFalcon.getStatorCurrent()>ArmConstants.kStallCurrent);
    armPositionEntry.setDouble(ArmFalcon.getSelectedSensorPosition()/ArmConstants.kSensorCountPerRevolution);
    armPowerEntry.setDouble(ArmFalcon.getStatorCurrent());
    if (!reverseLimit.get()&&!ingoreArmFwdLimitEntry.getBoolean(false)) {
      if(p_power>0){
        ArmFalcon.set(ControlMode.PercentOutput, 0);
      }
      else{
         ArmFalcon.set(ControlMode.PercentOutput, p_power);
      }
      ArmFalcon.setSelectedSensorPosition(ArmConstants.kReverseRotationCount);
    } else if ((extPos > Math.sqrt(Math.pow(elvPos * (ArmConstants.maxHorizontalPos+30)/ArmConstants.baseLength,2) + Math.pow(ArmConstants.maxHorizontalPos +30,2)) || ((ArmFalcon.getStatorCurrent()>powerLimitEntry.getDouble(ArmConstants.kStallCurrent))&&!ingoreArmFwdLimitEntry.getBoolean(false)))){
      if(p_power<0){ // tune maxhorizontalpos
        ArmFalcon.set(ControlMode.PercentOutput, 0);
        m_maxArmPosition = ArmFalcon.getSelectedSensorPosition();
      }
      else { // ----- add an else if statement that checks if elvPower < 0 and then set ArmFalcon to -elvPower if it's true
         ArmFalcon.set(ControlMode.PercentOutput, p_power);
      }
    } 
    else {
      ArmFalcon.set(ControlMode.PercentOutput, p_power);
    }
  }

  public double getArmDistance(){
    return -(ArmFalcon.getSelectedSensorPosition()/ArmConstants.kEncoderTicksPerRotation*1.43*Math.PI)+41;
  }

  public double getMaxArmPosition(double elevatorPosition){
    return Math.sqrt(Math.pow(elevatorPosition/ArmConstants.kEncoderTicksPerRotation*1.43*Math.PI * (ArmConstants.maxHorizontalPos+30)/ArmConstants.baseLength,2) + Math.pow(ArmConstants.maxHorizontalPos +30,2));
  }
  // public boolean homeArm(){
  //   if(homeArmBottom()){
  //     return homeArmTop();
  //   }
  //   else{
  //     return homeArmBottom();
  //   }
  // }

  // public boolean homeArmBottom(){
  //   if(reverseLimit.get()){
  //     ArmFalcon.setSelectedSensorPosition(ArmConstants.kReverseRotationCount);
  //     setPower(0, 0);
  //     return true;
  //   }
  //   else{
  //     setPower(-.5, 0);
  //     return false;
  //   }
  // }

  // public boolean homeArmTop(){
  //   if(ArmFalcon.getStatorCurrent()>ArmConstants.kStallCurrent){
  //     ArmFalcon.configForwardSoftLimitThreshold(ArmFalcon.getSelectedSensorPosition());
  //     setPower(0, 0);
  //     return true;
  //   }
  //   else{
  //     setPower(.5, 0);
  //     return false;
  //   }
  // }

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
