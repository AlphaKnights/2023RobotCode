// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Java Libraries
import java.io.IOException;
import java.util.Optional;

// Robot Constants
import frc.robot.commands.Constants.VisionConstants;

// FRC Imports
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;

// Photonvision library imports
// Docs: https://docs.photonvision.org/en/latest/docs/examples/apriltag.html
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PoseEstimator extends CommandBase {
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;

    /** Creates a new Vision command object **/
    public PoseEstimator() {
        // Initialize PhotonVision Camera
        // gettings data from a PhotonCamera - have not completed
        // https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html#constructing-a-photoncamera
        photonCamera = new PhotonCamera(VisionConstants.photonCamName);
    
        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            // Create PhotonVision pose estimator; PoseStrategy is subject to change
            photonPoseEstimator = new PhotonPoseEstimator(
                            fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, photonCamera, VisionConstants.robotToCam);
        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't have the tags
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
        }
    }
    
    // Read https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
    // for more information on its returns and usage
    /** Estimate robot's field position from AprilTags. Returns the estimted pose (Pose3d) and timestamp of estimation (double) **/
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}

/* Thanks Commodors - This should go in a Swerve class (when it is created)
    public void updateOdometry(){
        poseEstimator.update(getYaw(), getModulePositions());

        Optional<EstimatedRobotPose> result = photonPose.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());

        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
            field.getObject("Estimated Vision Position").setPose(camPose.estimatedPose.toPose2d());
        } else {
            field.getObject("Estimated Vision Position").setPose(new Pose2d(-100, -100, new Rotation2d()));
        }

        field.getObject("Actual Pos").setPose(getPose());
        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }
*/