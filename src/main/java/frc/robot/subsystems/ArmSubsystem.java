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
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  //Config the falcon and the limit switches
  TalonFX armFalcon = new TalonFX(ArmConstants.kArmFalconID);
  TalonFXConfiguration armConfig = new TalonFXConfiguration();
  DigitalInput reverseLimit = new DigitalInput(ArmConstants.kLimitSwitchPort);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    //Sets Encoder limits
    armConfig.forwardSoftLimitEnable = false;
    armConfig.reverseSoftLimitEnable = false;
    armConfig.forwardSoftLimitThreshold = ArmConstants.kFowardRotationCount;
    armConfig.reverseSoftLimitThreshold = ArmConstants.kReverseRotationCount;
    armFalcon.configAllSettings(armConfig);
    armFalcon.setNeutralMode(NeutralMode.Brake);
    armFalcon.setSelectedSensorPosition(ArmConstants.kReverseRotationCount);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * 
   * @param p_power the power to set the arm to, between -1 and 1. It will refuse to move in the reverse direction if the limit switch is pressed.
   */
  public void setPower(double p_power) {
    //Checks the limit switch and if triggered, sets the encoder to the reverse limit to the reverse limit and stops the motor
    // if (reverseLimit.get()) {
    //   if(p_power<0){
    //     armFalcon.set(ControlMode.PercentOutput, 0);
    //   }
    //   armFalcon.set(ControlMode.PercentOutput, 0);
    //   armFalcon.setSelectedSensorPosition(ArmConstants.kReverseRotationCount);
    // } else {
      armFalcon.set(ControlMode.PercentOutput, p_power);
    // }
  }

  /**
   * 
   * @param p_position the position to move the arm to, in encoder counts.
   */
  public void goToPosition(double p_position) {
    //Checks the limit switch and if triggered, sets the encoder to the reverse limit to the reverse limit
    if(reverseLimit.get()){
      armFalcon.setSelectedSensorPosition(ArmConstants.kReverseRotationCount);
    }
    armFalcon.set(ControlMode.Position, p_position);
  }
  
  /**
   * Gets the arm Falcon object, for debugging use only.
   */
  public TalonFX getArmFalcon(){
    return armFalcon;
  }

  /**
   * Gets the arm limit switch object, for debugging use only.
   */
  public DigitalInput getLimitSwitch(){
    return reverseLimit;
  }
}
