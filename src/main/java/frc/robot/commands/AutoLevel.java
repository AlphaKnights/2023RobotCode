// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalanceConstants;
import frc.robot.Constants.NetworkTableConstants;
import frc.robot.subsystems.DriveSubsystem;
import com.kauailabs.navx.frc.AHRS;

public class AutoLevel extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  AHRS navX;
  boolean onStation;
  boolean docked;
  Timer timer;
  NetworkTableEntry navxX = NetworkTableConstants.kDriveTable.getEntry("X");
  NetworkTableEntry navxY = NetworkTableConstants.kDriveTable.getEntry("Y");
  NetworkTableEntry navxRot = NetworkTableConstants.kDriveTable.getEntry("Rot");
  /** Creates a new ChangePistionState. */
  public AutoLevel(DriveSubsystem p_driveSubsystem) {
    timer = new Timer();
    navxX.setDefaultDouble(0.0);
    navxY.setDefaultDouble(0.0);
    navxRot.setDefaultDouble(0.0);
    // Use addRequirements() here to declare subsystem dependencies.
    this.navX = p_driveSubsystem.getGyro();
    onStation = false;
    docked = false;
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
    navxRot.setDouble(navX.getPitch());
    navxX.setDouble(navX.getAltitude());
    navxY.setDouble(navX.getDisplacementY());
    System.out.println(navX.getPitch());
    if (onStation && (Math.abs(navX.getPitch()) > 2 && docked)){ System.out.println("if");
        docked = false;
        timer.stop();
        timer.reset();
    }
    else if (onStation && Math.abs(navX.getPitch())<2){
        m_driveSubsystem.drive(0,0,0,true,false);
        docked = true;
        System.out.println("on");
      }
    else if (!onStation && Math.abs(navX.getPitch()) < 10){
        m_driveSubsystem.drive(0.2, 0.0, 0, true, false);
    System.out.println("fwd1");
      }
    else if(!onStation) {
      onStation = true;
    }
    else if (onStation){
        if (Math.abs(navX.getPitch()) <= 2&&navX.getAltitude()>=BalanceConstants.fieldAltitude + BalanceConstants.altitudeError+8){
            System.out.println(navX.getAltitude());
            timer.start();
            onStation = true;
            m_driveSubsystem.drive(0,0,0,true,false);
            System.out.println("on");
          }
        else{
            m_driveSubsystem.drive(MathUtil.clamp(Math.abs(-.25*(navX.getPitch())/navX.getPitch()), -.2, .2),0, 0, true, false);
            System.out.println("fwd2");
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