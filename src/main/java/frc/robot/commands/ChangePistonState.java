// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.PistonState;
import frc.robot.subsystems.ClawSubsystem;

public class ChangePistonState extends InstantCommand {
  ClawSubsystem clawSubsystem;
  PistonState state;
  /** Creates a new ChangePistionState. */
  public ChangePistonState(PistonState newState, ClawSubsystem _clawSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    state = newState;
    clawSubsystem = _clawSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawSubsystem.setPistonState(state);
  }
}
