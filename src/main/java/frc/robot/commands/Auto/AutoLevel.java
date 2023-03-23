
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoBalancerConstants;
import frc.robot.subsystems.DriveSubsystem;
import com.kauailabs.navx.frc.AHRS;

public class AutoLevel extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final AHRS navX;
  private static boolean onStation = false;
  private static double pitch = 0;
  private static double speed = 0.2;
  private static double processVariable_P = 4;
  private static double lastAngle = 0;
  public AutoLevel(DriveSubsystem p_driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.navX = p_driveSubsystem.getGyro();
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
    pitch = navX.getPitch()+AutoBalancerConstants.fieldError;
    if(onStation){
      //If the robot isn't passed the set angle, set the speed to the inverse of the process variable (1/pv)
      if (Math.abs(pitch) > AutoBalancerConstants.setAngle+AutoBalancerConstants.tolerance) speed = Math.copySign(pitch, 1/processVariable_P);
      else speed = 0;
      //If passed the set angle, increase the process variable to decrease the speed of the robot
      if (Math.abs(pitch)>AutoBalancerConstants.setAngle+AutoBalancerConstants.tolerance && lastAngle<=AutoBalancerConstants.setAngle+AutoBalancerConstants.tolerance)processVariable_P *= AutoBalancerConstants.kP;
      m_driveSubsystem.drive(speed,0, 0, true, false);
    }
    else{
      if (Math.abs(pitch) < 10) m_driveSubsystem.drive(AutoBalancerConstants.fieldSpeed, 0.0, 0, true, false);
      else onStation = true;
    }
    lastAngle = Math.abs(pitch);
  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0,0,0,true,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}