// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.PistonState;

public class ClawSubsystem extends SubsystemBase {
  //Sets up the pneumatics hub for the claw
  PneumaticHub clawHub = new PneumaticHub(ClawConstants.kRevPneumaticPort);
  //Sets up the pistons for the claw
  DoubleSolenoid piston1 = new DoubleSolenoid(ClawConstants.kRevPneumaticPort, PneumaticsModuleType.REVPH, ClawConstants.kPiston1FwdPort, ClawConstants.kPiston1RevPort);
  DoubleSolenoid piston2 = new DoubleSolenoid(ClawConstants.kRevPneumaticPort, PneumaticsModuleType.REVPH, ClawConstants.kPiston2FwdPort, ClawConstants.kPiston2RevPort);
  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the state of the pistons
   * 
   * @param p_state state the claw should be, OPEN, HALF, CLOSED, or OFF
   */
  public void setPistonState(PistonState p_state) {
    switch (p_state) {
      case OPEN:
        piston1.set(DoubleSolenoid.Value.kForward);
        piston2.set(DoubleSolenoid.Value.kForward);
        break;
      case HALF:
        piston1.set(DoubleSolenoid.Value.kReverse);
        piston2.set(DoubleSolenoid.Value.kForward);
        break;
      case CLOSED:
        piston1.set(DoubleSolenoid.Value.kReverse);
        piston2.set(DoubleSolenoid.Value.kReverse);
        break;
      case OFF:
        piston1.set(DoubleSolenoid.Value.kOff);
        piston2.set(DoubleSolenoid.Value.kOff);
        break;
    }
  }
}