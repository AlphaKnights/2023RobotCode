// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ArmMove extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  // private ElevatorSubsystem m_ElevatorSubsystem;
  private Joystick m_operatorRightJoystick;
  // private TalonFX f;
  private double pos = 0;
  /** Creates a new ArmMove. */
  public ArmMove(ArmSubsystem p_armSubsystem, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_operatorRightJoystick = joystick;
    // m_ElevatorSubsystem = p_elevatorSubsystem;
    m_armSubsystem = p_armSubsystem;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // f = m_ElevatorSubsystem.getElevatorFalcon();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setPower(m_operatorRightJoystick.getY()*m_operatorRightJoystick.getThrottle(), pos);
    System.out.println(pos);
  }

  public void setpos(double p){
    pos = p;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
