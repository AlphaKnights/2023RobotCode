// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorGoToPosition extends CommandBase {
  ElevatorSubsystem m_ElevatorSubsystem;
  private final double numOfRot;
  private final double sensorPos;
  int count = 0;
  /** Creates a new ElevatorGoToPosition. */
  public ElevatorGoToPosition(ElevatorSubsystem p_ElevatorSubsystem, double p_numOfRot) {
    numOfRot = p_numOfRot;
    sensorPos =  numOfRot* ElevatorConstants.kSensorCountPerRevolution;
    m_ElevatorSubsystem = p_ElevatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(sensorPos);
    m_ElevatorSubsystem.setToPosition(sensorPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ElevatorSubsystem.goToPosition();
    count++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_ElevatorSubsystem.getPosition()>sensorPos-10&&m_ElevatorSubsystem.getPosition()<sensorPos+10;
    if(count>1000){
      return true;
    }
    return false;
  }
}
