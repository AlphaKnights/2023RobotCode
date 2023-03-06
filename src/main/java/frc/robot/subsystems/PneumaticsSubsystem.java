// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;
import frc.robot.Constants.NetworkTableConstants;
import frc.robot.Constants.PistonState;

public class PneumaticsSubsystem extends SubsystemBase {
  PneumaticHub pneumaticHub = new PneumaticHub(PneumaticConstants.kRevPneumaticPort);
  //Sets up the pistons for the claw
  DoubleSolenoid clawPiston = new DoubleSolenoid(PneumaticConstants.kRevPneumaticPort, PneumaticsModuleType.REVPH, PneumaticConstants.kClawFwdPort, PneumaticConstants.kClawRevPort);
  DoubleSolenoid wristPiston = new DoubleSolenoid(PneumaticConstants.kRevPneumaticPort, PneumaticsModuleType.REVPH, PneumaticConstants.kWristFwdPort, PneumaticConstants.kWristRevPort);
  //Sets up SmartDashboard for the status of the compressor and the double solenoids
  
  NetworkTableEntry pressureEntry = NetworkTableConstants.kPneumaticTable.getEntry("Pressure");
  // NetworkTableEntry elevatorIEntry = NetworkTableConstants.kPneumaticTable.getEntry("I");
  // NetworkTableEntry elevatorDEntry = NetworkTableConstants.kPneumaticTable.getEntry("D");
  /** Creates a new ClawSubsystem. */
  public PneumaticsSubsystem() {
    pressureEntry.setDefaultDouble(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pressureEntry.setDouble(pneumaticHub.getPressure(0));
  }

  /**
   * Sets the state of the pistons
   * 
   * @param p_state state the claw should be, OPEN, HALF, CLOSED, or OFF
   */
  public void setClawState(PistonState p_state) {
    switch (p_state) {
      case OPEN:
        clawPiston.set(DoubleSolenoid.Value.kForward);
        break;
      case CLOSED:
        clawPiston.set(DoubleSolenoid.Value.kReverse);
        break;
      case OFF:
        clawPiston.set(DoubleSolenoid.Value.kOff);
        break;
    }
  }
  
  /**
   * Gets the state of the claw piston
   * @return PistonState
   */
  public PistonState getClawState(){
    switch (clawPiston.get()){
      case kForward:
        return PistonState.OPEN;
      case kReverse:
        return PistonState.CLOSED;
      case kOff:
        return PistonState.OFF;
      default:
        return PistonState.OFF;
    }
  }

  /**
   * Sets the state of the pistons
   * 
   * @param p_state state the wrist should be, UP, DOWN, or OFF
   */
  public void setWristState(PistonState p_state) {
    switch (p_state) {
      case OPEN:
        wristPiston.set(DoubleSolenoid.Value.kForward);
        break;
      case CLOSED:
        wristPiston.set(DoubleSolenoid.Value.kReverse);
        break;
      case OFF:
        wristPiston.set(DoubleSolenoid.Value.kOff);
        break;
    }
  }

  /**
   * Gets the state of the wrist piston
   * @return PistonState
   */
  public PistonState getWristState(){
    switch (wristPiston.get()){
      case kForward:
        return PistonState.OPEN;
      case kReverse:
        return PistonState.CLOSED;
      case kOff:
        return PistonState.OFF;
      default:
        return PistonState.OFF;
    }
  }

  /**
   * Sets the pressure for the compressor
   * @param p_min minimum pressure
   * @param p_max maximum pressure
   */
  public void setCompressorPressure(double p_min, double p_max){
    pneumaticHub.enableCompressorAnalog(p_min,p_max);
  }

  /**
   * Disables the compressor
   */
  public void disableCompressor(){
    pneumaticHub.disableCompressor();
  }

  /**
   * Toggles the compressor
   */
  public void toggleCompressor(){
    if(pneumaticHub.getCompressor()){
      disableCompressor();
    }
    else{
      System.out.println("Enabling!");
      setCompressorPressure(PneumaticConstants.kClawMinPressure, PneumaticConstants.kClawMaxPressure);
    }
  }

  /**
   * Toggles the compressor
   * @param p_min minimum pressure
   * @param p_max maximum pressure
   */
  public void toggleCompressor(double p_min, double p_max){
    if(pneumaticHub.getCompressor()){
      disableCompressor();
    }
    else{
      setCompressorPressure(p_min, p_max);
    }
  }

  /**
   * Gets the Pneumatic Hub object, for debugging use only. 
   * @return
   */
  public PneumaticHub getPneumaticHub() {
      return pneumaticHub;
  }

  /**
   * Gets the claw piston object, for debugging use only. 
   * @return
   */
  public DoubleSolenoid getClawPiston() {
      return clawPiston;
  }

  /**
   * Gets the wrist piston object, for debugging use only. 
   * @return
   */
  public DoubleSolenoid getWristPiston() {
      return wristPiston;
  }
}
