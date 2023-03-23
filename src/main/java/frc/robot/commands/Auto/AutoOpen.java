// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.PistonState;
import frc.robot.subsystems.PneumaticsSubsystem;

public class AutoOpen extends InstantCommand {
  /** Creates a new AutoOpen. */
  private PneumaticsSubsystem m_pneumaticsSubsystem;
  public AutoOpen(PneumaticsSubsystem p_pneumaticsSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pneumaticsSubsystem = p_pneumaticsSubsystem;
    addRequirements(m_pneumaticsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pneumaticsSubsystem.setCompressorPressure(80, 90);
    m_pneumaticsSubsystem.setClawState(PistonState.OPEN);}
    
}
