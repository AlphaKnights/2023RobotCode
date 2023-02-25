// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ResetEncodersCommand extends InstantCommand {
  DriveSubsystem driveSubsystem;
  /** Creates a new ResetEncoders. */
  public ResetEncodersCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Resets the encoders
    driveSubsystem.resetEncoders();
  }
}
