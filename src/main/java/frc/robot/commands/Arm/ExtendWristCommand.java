// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PistonState;
import frc.robot.subsystems.PneumaticsSubsystem;

public class ExtendWristCommand extends CommandBase {
  PneumaticsSubsystem m_pneumaticsSubsystem;
  /** Creates a new WristCommand. */
  public ExtendWristCommand(PneumaticsSubsystem p_pneumaticsSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pneumaticsSubsystem = p_pneumaticsSubsystem;
    addRequirements(m_pneumaticsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pneumaticsSubsystem.setWristState(PistonState.OPEN);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pneumaticsSubsystem.setWristState(PistonState.OFF);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
