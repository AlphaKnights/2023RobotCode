// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.VisionConstants;

// import java.io.IOException;
// import java.util.Optional;

// // FRC Imports
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.wpilibj.DriverStation;

// // Photonvision library imports
// // Docs: https://docs.photonvision.org/en/latest/docs/examples/apriltag.html
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;

// public class LocalizationSubsystem extends SubsystemBase {
// 	/** Creates a new LocalizationSubsystem. */
// 	private static PhotonCamera photonCamera;
//     private static PhotonPoseEstimator photonPoseEstimator;
// 	private static AprilTagFieldLayout fieldLayout;


// 	public LocalizationSubsystem() {
// 		try{
// 			fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
// 		}
// 		catch(IOException e){
// 			DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
//             photonPoseEstimator = null;
// 		}
// 		PoseEstimator();
// 	}

// 	@Override
// 	public void periodic() {
//     getEstimatedGlobalPose(new Pose2d());
// 		// This method will be called once per scheduler run
// 	}

//     /** Creates a new Vision command object **/
//     public static void PoseEstimator() {
//         photonCamera = new PhotonCamera(VisionConstants.kCameraName);
// 			//fieldLayout.getTagPose(2).get().toPose2d();
//             photonPoseEstimator = new PhotonPoseEstimator(
//                             fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, photonCamera, VisionConstants.kCameraOffset);
//     }

//     /** Estimate robot's field position from AprilTags. Returns the estimted pose (Pose3d) and timestamp of estimation (double) **/
//     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
// 		if (photonPoseEstimator == null) {
// 			System.out.println("Not init!");
//             // The field layout failed to load, so we cannot estimate poses.
//             return Optional.empty();
//         }
//         photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
//         return photonPoseEstimator.update();
//     }
// 	public static Pose2d CalculateAprilTagDesiredPosition(int AprilTagID){//Change 10 to how far back you want to be from tag (should be held in constants)
// 		return new Pose2d(fieldLayout.getTagPose(AprilTagID).get().getX(), fieldLayout.getTagPose(AprilTagID).get().getY()-10, new Rotation2d(90));
// 	}
// 	public static int ClosestAprilTagID(Pose2d initialPose){//add functionality for the other tags
// 		Math.sqrt(Math.pow(fieldLayout.getTagPose(1).get().getX()-initialPose.getX(),2) + Math.pow(fieldLayout.getTagPose(1).get().getY()-initialPose.getY(),2));
// 		return 1; // fix this to reflect correct ID
// 	}
// }