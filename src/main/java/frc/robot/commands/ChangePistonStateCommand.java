// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.PistonState;
import frc.robot.subsystems.PneumaticsSubsystem;

public class ChangePistonStateCommand extends InstantCommand {
  PneumaticsSubsystem m_clawSubsystem;
  PistonState m_state;
  /** Creates a new ChangePistionState. */
  public ChangePistonStateCommand(PistonState p_newState, PneumaticsSubsystem p_clawSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_state = p_newState;
    m_clawSubsystem = p_clawSubsystem;
    addRequirements(m_clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Sets the position of the pistons
    m_clawSubsystem.setClawState(m_state);
  }
}
