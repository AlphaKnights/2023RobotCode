// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class NavXZeroCommand extends InstantCommand {
  DriveSubsystem m_driveSubsystem;
  /** Creates a new NavXZero. */
  public NavXZeroCommand(DriveSubsystem p_driveSubsystem) {
    m_driveSubsystem = p_driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.zeroHeading();
  }
}
