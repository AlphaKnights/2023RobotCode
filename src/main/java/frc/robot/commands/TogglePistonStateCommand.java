// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.PistonState;
import frc.robot.subsystems.PneumaticsSubsystem;

public class TogglePistonStateCommand extends InstantCommand {
  PneumaticsSubsystem m_pnuematicsSubsystem;
  /** Creates a new ChangePistionState. */
  public TogglePistonStateCommand(PneumaticsSubsystem p_clawSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pnuematicsSubsystem = p_clawSubsystem;
    addRequirements(m_pnuematicsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Sets the position of the pistons
    if(m_pnuematicsSubsystem.getClawState() == PistonState.OPEN){
      m_pnuematicsSubsystem.setClawState(PistonState.CLOSED);
    }
    else{
      m_pnuematicsSubsystem.setClawState(PistonState.OPEN);}
  }
}
