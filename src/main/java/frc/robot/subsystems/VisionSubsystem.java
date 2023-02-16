// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Robot Constants
import frc.robot.commands.Constants.VisionConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Photonvision library imports
// Docs: https://docs.photonvision.org/en/latest/docs/examples/apriltag.html
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

public class VisionSubsystem extends SubsystemBase {
	private static PhotonCamera camera = new PhotonCamera(VisionConstants.photonCamName);
    /** Creates a new Vision Subsystem object **/
    public VisionSubsystem() {}

    @Override
    public void periodic() {
		// Get the latest result from PhotonVision
		var result = camera.getLatestResult();
		if (!result.hasTargets()) return;
		//List<PhotonTrackedTarget> targets = result.getTargets();
		PhotonTrackedTarget target = result.getBestTarget();
    }
}
