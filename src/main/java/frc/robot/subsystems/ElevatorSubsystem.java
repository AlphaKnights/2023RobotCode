// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  TalonFX elevatorFalcon = new TalonFX(ElevatorConstants.kElevatorFalconID);
  TalonFXConfiguration armConfig = new TalonFXConfiguration();
  DigitalInput reverseLimit = new DigitalInput(ElevatorConstants.kLimitSwitchPort);

  /** Creates a new ArmSubsystem. */
  public ElevatorSubsystem() {
    armConfig.forwardSoftLimitEnable = true;
    armConfig.reverseSoftLimitEnable = true;
    armConfig.forwardSoftLimitThreshold = ElevatorConstants.kFowardVerticalCount;
    armConfig.reverseSoftLimitThreshold = ElevatorConstants.kReverseVerticalCount;
    elevatorFalcon.configAllSettings(armConfig);
    elevatorFalcon.setSelectedSensorPosition(ElevatorConstants.kReverseVerticalCount);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double Power) {
    if (reverseLimit.get()) {
      elevatorFalcon.set(ControlMode.PercentOutput, 0);
      elevatorFalcon.setSelectedSensorPosition(ElevatorConstants.kReverseVerticalCount);
    } else {
      elevatorFalcon.set(ControlMode.PercentOutput, Power);
    }
  }

  public void goToPosition(double position) {
    if(reverseLimit.get()){
      elevatorFalcon.setSelectedSensorPosition(ElevatorConstants.kReverseVerticalCount);
    }
    elevatorFalcon.set(ControlMode.Position, position);
  }
}
