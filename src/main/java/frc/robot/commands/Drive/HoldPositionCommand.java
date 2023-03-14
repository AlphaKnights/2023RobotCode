// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class HoldPositionCommand extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  // private boolean m_isFinished = false;
  /** Creates a new HoldPosition. */
  public HoldPositionCommand(DriveSubsystem p_driveSubsystem) {
    m_driveSubsystem = p_driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Sets the swerve drive to an x pattern
    m_driveSubsystem.setXFormation();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stopXFormation();
    System.out.println("End");
    // m_isFinished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
