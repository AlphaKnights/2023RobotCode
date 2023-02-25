// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  //Config the falcon and the limit switches
  TalonFX elevatorFalcon = new TalonFX(ElevatorConstants.kElevatorFalconID);
  TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
  DigitalInput reverseLimit = new DigitalInput(ElevatorConstants.kLimitSwitchPort);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorConfig.forwardSoftLimitEnable = false;
    elevatorConfig.reverseSoftLimitEnable = false;
    elevatorConfig.forwardSoftLimitThreshold = ElevatorConstants.kFowardVerticalCount;
    elevatorConfig.reverseSoftLimitThreshold = ElevatorConstants.kReverseVerticalCount;
    elevatorFalcon.configAllSettings(elevatorConfig);
    elevatorFalcon.setNeutralMode(NeutralMode.Brake);
    elevatorFalcon.setSelectedSensorPosition(ElevatorConstants.kReverseVerticalCount);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * 
   * @param p_power the power to set the elevator to, between -1 and 1. It will refuse to move in the reverse direction if the limit switch is pressed.
   */
  public void setPower(double p_power) {
    // if (reverseLimit.get()) {
    //   if(p_power<0){
    //     elevatorFalcon.set(ControlMode.PercentOutput, 0);
    //   }
    //   elevatorFalcon.setSelectedSensorPosition(ElevatorConstants.kReverseVerticalCount);
    // } else {
      elevatorFalcon.set(ControlMode.PercentOutput, p_power);
    // }
  }

  /**
   * 
   * @param p_position the position to move the elevator to, in encoder counts.
   */
  public void goToPosition(double p_position) {
    if(reverseLimit.get()){
      elevatorFalcon.setSelectedSensorPosition(ElevatorConstants.kReverseVerticalCount);
    }
    elevatorFalcon.set(ControlMode.Position, p_position);
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
