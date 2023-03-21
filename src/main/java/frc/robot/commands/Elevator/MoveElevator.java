// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevator extends CommandBase {
  private ArmSubsystem m_armSubsystem;
  private ElevatorSubsystem m_ElevatorSubsystem;
  private Joystick m_operatorRightJoystick;
  private ArmMove m;
  /** Creates a new ArmMove. */
  public MoveElevator(ElevatorSubsystem p_elevatorSubsystem, Joystick joystick, ArmMove c) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_operatorRightJoystick = joystick;
    m_ElevatorSubsystem = p_elevatorSubsystem;
    m = c;
    addRequirements(m_ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ElevatorSubsystem.setPower(m_operatorRightJoystick.getY()*m_operatorRightJoystick.getThrottle(), null);
    m.setpos(m_ElevatorSubsystem.getPosition());
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
