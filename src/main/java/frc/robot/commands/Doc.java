// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants.BalanceConstants;

public class Doc extends InstantCommand {
  DriveSubsystem m_driveSubsystem;
  AHRS navX;
  boolean onStation;
  /** Creates a new ChangePistionState. */
  public Doc(AHRS navX, DriveSubsystem p_driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.navX = navX;
    onStation = false;
    m_driveSubsystem = p_driveSubsystem;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Sets the position of the pistons
  }

  public void execute() {
    if (onStation && (Math.abs(navX.getYaw()) > 2)){
        onStation = false;
    }
    else if (onStation){
        m_driveSubsystem.drive(0,0,0,true,false);
    }
    else if (!onStation && navX.getAltitude()<=BalanceConstants.fieldAltitude + BalanceConstants.altitudeError && Math.abs(navX.getYaw()) < 10){
        m_driveSubsystem.drive(0, 0.5, 0, true, false);
    }
    else if (!onStation){
        m_driveSubsystem.drive(0,navX.getYaw()/180/5, 0, true, false);
        if (Math.abs(navX.getYaw()) <= 2){
            onStation = true;
            m_driveSubsystem.drive(0,0,0,true,false);
        }
    }

  }
}