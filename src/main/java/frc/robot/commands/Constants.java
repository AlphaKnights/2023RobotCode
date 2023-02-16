package frc.robot.commands;

// WPI Math libraries
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {
    public static class VisionConstants {
        // Camera mounted facing forward, half a meter forward of center, half a meter up, subject to change
        public static final Transform3d robotToCam = new Transform3d(
                                                new Translation3d(0.5, 0.0, 0.5),
                                                new Rotation3d(0, 0,0)
                                            );
        // Change the camera name to the one in the PhotonVision Dashboard settings panel
        public static final String photonCamName = "6695VisionSource0";
    }
}
