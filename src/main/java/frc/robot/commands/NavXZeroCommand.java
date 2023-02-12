// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class NavXZeroCommand extends CommandBase {
  DriveSubsystem driveSubsystem;
  boolean executed = false;
  /** Creates a new NavXZero. */
  public NavXZeroCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Zero navx when executed
    driveSubsystem.zeroHeading();
    //Set executed true so that the NavX is only zeroed once
    executed = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Once the execute program is called isFinished returns true. 
    //This makes it so that the command is only run once per button press
    if (executed) {
      executed = false;
      return true;
    } else {
      return false;
    }
  }
}
