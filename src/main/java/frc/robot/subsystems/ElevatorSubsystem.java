// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
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
  private double m_maxElevatorVerticalPosition = ElevatorConstants.kDefaultMaxFowardVerticalRotation*ElevatorConstants.kSensorCountPerRevolution;
  
  NetworkTableEntry elevatorPEntry = NetworkTableConstants.kElevatorTable.getEntry("P");
  NetworkTableEntry elevatorIEntry = NetworkTableConstants.kElevatorTable.getEntry("I");
  NetworkTableEntry elevatorDEntry = NetworkTableConstants.kElevatorTable.getEntry("D");
  NetworkTableEntry elevatorFwdLimit = NetworkTableConstants.kElevatorTable.getEntry("FwdLimit");
  NetworkTableEntry elevatorRvsLimit = NetworkTableConstants.kElevatorTable.getEntry("RvsLimit");
  NetworkTableEntry elevatorPosition = NetworkTableConstants.kElevatorTable.getEntry("Position");
  NetworkTableEntry elevatorPower = NetworkTableConstants.kElevatorTable.getEntry("Power");
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorPEntry.setDefaultDouble(ElevatorConstants.kP);
    elevatorIEntry.setDefaultDouble(ElevatorConstants.kI);
    elevatorDEntry.setDefaultDouble(ElevatorConstants.kD);
    elevatorFwdLimit.setDefaultBoolean(false);
    elevatorRvsLimit.setDefaultBoolean(false);
    elevatorPosition.setDefaultDouble(ElevatorConstants.kMaxReverseVerticalRotationCount);
    elevatorPower.setDefaultDouble(0);

    elevatorConfig.forwardSoftLimitEnable = false;
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
    // if(NetworkTableConstants.DEBUG){
    //   if(i>=25){
    //     i = 1;
    //   }
    //   else{
    //     i++;
    //   }
    // }
  }

  /**
   * 
   * @param p_power the power to set the elevator to, between -1 and 1. It will refuse to move in the reverse direction if the limit switch is pressed.
   */
  public void setPower(double p_power) {
    // System.out.println("p");
    elevatorFwdLimit.setBoolean(!forwardLimit.get());
    elevatorRvsLimit.setBoolean(!reverseLimit.get());
    elevatorPosition.setDouble(elevatorFalcon.getSelectedSensorPosition()/ElevatorConstants.kSensorCountPerRevolution);
    elevatorPower.setDouble(elevatorFalcon.getStatorCurrent());
    if (reverseLimit.get()) {
      // System.out.println("Rev");
      if(p_power<0){
        elevatorFalcon.set(ControlMode.PercentOutput, 0);
      }
      else{
        elevatorFalcon.set(ControlMode.PercentOutput, p_power);
      }
      elevatorFalcon.setSelectedSensorPosition(ElevatorConstants.kMaxReverseVerticalRotationCount);
    } else if (!forwardLimit.get()){//Normally Open, so if it's not pressed, it's true
      // System.out.println("Fwd");
      if(p_power>0){
        elevatorFalcon.set(ControlMode.PercentOutput, 0);
      }
      else{
        elevatorFalcon.set(ControlMode.PercentOutput, p_power);
      }
      // elevatorFalcon.setSelectedSensorPosition(ElevatorConstants.kFowardVerticalCount);
    } else {
     elevatorFalcon.set(ControlMode.PercentOutput, p_power);
     if (p_power <0){

     }
    }
  }

  public void goToPosition(){
    System.out.println(elevatorPID.calculate(elevatorFalcon.getSelectedSensorPosition()));
  }

  // public double getElvPosition(){
  //   return elevatorFalcon.getSelectedSensorPosition()/ElevatorConstants.kSensorCountPerRevolution;
  // }

  /**
   * 
   * @param p_position the position to move the elevator to, in encoder counts.
   */
  public void setToPosition(double p_position) {
    elevatorPID.setP(elevatorPEntry.getDouble(ElevatorConstants.kP));
    elevatorPID.setI(elevatorIEntry.getDouble(ElevatorConstants.kI));
    elevatorPID.setD(elevatorDEntry.getDouble(ElevatorConstants.kD));
    // elevatorPID.
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
    System.out.println(elevatorPID.calculate(elevatorFalcon.getSelectedSensorPosition(), p_position));
    // elevatorPID.
    // elevatorFalcon.set(ControlMode.Position, elevatorPID.calculate(elevatorFalcon.getSelectedSensorPosition(), p_position*ElevatorConstants.kF));
    // setPower(MathUtil.clamp(,-1*ElevatorConstants.kPeakOutput,ElevatorConstants.kPeakOutput));
    // elevatorFalcon.set(ControlMode.Position, p_position);
  }

  public boolean homeElevator(){
    // return true;
    if(homeElevatorBottom()){
      return homeElevatorTop();
    }
    else{
      return homeElevatorBottom();
    }
  }

  public boolean homeElevatorBottom(){
    // return true;
    if(reverseLimit.get()){
      // elevatorFalcon.setSelectedSensorPosition(ElevatorConstants.kMaxReverseVerticalRotationCount);
      // setPower(0);
      return true;
    }
    else{
      setPower(-.5);
      return false;
    }
  }

  public boolean homeElevatorTop(){
    // return true;
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
  public DigitalInput getFLimitSwitch(){
    return forwardLimit;
  }
}
