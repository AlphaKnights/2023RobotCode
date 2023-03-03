// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorPower extends CommandBase {
  ElevatorSubsystem m_ElevatorSubsystem;
  Joystick m_operatorLeftJoystick;
  /** Creates a new ElevatorGoToPosition. */
  public SetElevatorPower(ElevatorSubsystem p_ElevatorSubsystem, Joystick p_operatorLeftJoystick) {
    m_operatorLeftJoystick = p_operatorLeftJoystick;
    m_ElevatorSubsystem = p_ElevatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ElevatorSubsystem.setPower(m_operatorLeftJoystick.getY()*m_operatorLeftJoystick.getThrottle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_ElevatorSubsystem.getPosition()>sensorPos-10&&m_ElevatorSubsystem.getPosition()<sensorPos+10;
    return false;
  }
}
