// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase {
	/** Creates a new DriveTrain. */
	public DriveTrain() {}
	private final MAXSwerveModule m_frontLeft = new MAXSwerveModule( //CHANGE SMARTDASHBOARD INPUT TO DriveConstants.kFrontLeftChassisAngularOffset AND RESPECTIVE VARIABLES
      0,
      0,
      SmartDashboard.getNumber("FrontLeftAngle", 0));

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      0,
      0,
      SmartDashboard.getNumber("FrontRightAngle", 0));

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      0,
      0,
      SmartDashboard.getNumber("BackLeftAngle", 0));

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      0,
      0,
      SmartDashboard.getNumber("BackRightAngle", 0));

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void Goto(Pose2d targetPose, Pose2d currentPose){
		m_rearLeft.setDesiredState(new SwerveModuleState(1, currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle()));
	}
}
