// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalanceConstants;
import frc.robot.subsystems.DriveSubsystem;
import com.kauailabs.navx.frc.AHRS;

public class AutoLevel extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  AHRS navX;
  boolean onStation;
  Timer timer;
  /** Creates a new ChangePistionState. */
  public AutoLevel(DriveSubsystem p_driveSubsystem) {
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    this.navX = p_driveSubsystem.getGyro();
    onStation = false;
    m_driveSubsystem = p_driveSubsystem;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Sets the position of the pistons
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (onStation && (Math.abs(navX.getYaw()) > 2)){
        onStation = false;
        timer.stop();
        timer.reset();
    }
    else if (onStation){
        m_driveSubsystem.drive(0,0,0,true,false);
    }
    else if (!onStation && navX.getAltitude()<=BalanceConstants.fieldAltitude + BalanceConstants.altitudeError && Math.abs(navX.getYaw()) < 10){
        m_driveSubsystem.drive(0, 0.5, 0, true, false);
    }
    else if (!onStation){
        if (Math.abs(navX.getYaw()) <= 2){
            timer.start();
            onStation = true;
            m_driveSubsystem.drive(0,0,0,true,false);
        }
        else{
            m_driveSubsystem.drive(0,navX.getYaw()/180/5, 0, true, false);
        }
    }
  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0,0,0,true,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onStation && timer.get() > 1;
  }
}