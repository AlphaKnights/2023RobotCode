// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
    armConfig.forwardSoftLimitEnable = true;
    armConfig.reverseSoftLimitEnable = true;
    armConfig.forwardSoftLimitThreshold = ArmConstants.kFowardRotationCount;
    armConfig.reverseSoftLimitThreshold = ArmConstants.kReverseRotationCount;
    armFalcon.configAllSettings(armConfig);
    armFalcon.setSelectedSensorPosition(ArmConstants.kReverseRotationCount);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * 
   * @param Power the power to set the arm to, between -1 and 1. It will refuse to move in the reverse direction if the limit switch is pressed.
   */
  public void setPower(double Power) {
    if (reverseLimit.get()) {
      armFalcon.set(ControlMode.PercentOutput, 0);
      armFalcon.setSelectedSensorPosition(ArmConstants.kReverseRotationCount);
    } else {
      armFalcon.set(ControlMode.PercentOutput, Power);
    }
  }

  /**
   * 
   * @param position the position to move the arm to, in encoder counts.
   */
  public void goToPosition(double position) {
    if(reverseLimit.get()){
      armFalcon.setSelectedSensorPosition(ArmConstants.kReverseRotationCount);
    }
    armFalcon.set(ControlMode.Position, position);
  }
}
