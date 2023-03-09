// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoZeroCommand extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  AutoLevel levelCommand;
  Timer t;
  /** Creates a new NavXZero. */
  public AutoZeroCommand(DriveSubsystem p_driveSubsystem) {
    t=new Timer();
    t.reset();
    m_driveSubsystem = p_driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    t.start();
    m_driveSubsystem.zeroHeading();
    levelCommand.execute();
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    t.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_ElevatorSubsystem.getPosition()>sensorPos-10&&m_ElevatorSubsystem.getPosition()<sensorPos+10;
    return t.get()>1.5;
  }
}
