package frc.robot.Subsytems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/**
 * Vision class to interact with the Limelight camera.
 * This class can be used across different subsystems.
 */
public class Vision {

    private String limelightName;

    /**
     * Initializes the Vision system with the specified Limelight name.
     *
     * @param limelightName The network table name of your Limelight.
     */
    public Vision(String limelightName) {
        this.limelightName = limelightName;
    }

    /**
     * Checks if the Limelight has a valid target.
     *
     * @return True if a valid target is detected.
     */
    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    /**
     * Gets the latest pose estimation from the Limelight.
     *
     * @return A Pose2d object representing the robot's position and orientation.
     */
    public Pose2d getLatestPose() {
        double[] botpose = LimelightHelpers.getBotPose(limelightName);

        if (botpose.length < 6) {
            // Invalid data
            return null;
        }

        // botpose is an array of 6 elements: [X, Y, Z, Roll, Pitch, Yaw]
        // Convert X, Y from meters to Pose2d
        double x = botpose[0];
        double y = botpose[1];
        double yaw = botpose[5];

        return new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
    }

    /**
     * Gets the horizontal offset (tx) from the crosshair to the target.
     *
     * @return The horizontal offset in degrees.
     */
    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    /**
     * Gets the vertical offset (ty) from the crosshair to the target.
     *
     * @return The vertical offset in degrees.
     */
    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }

    /**
     * Gets the latency of the Limelight's pipeline.
     *
     * @return The pipeline latency in milliseconds.
     */
    public double getLatency() {
        return LimelightHelpers.getLatency_Pipeline(limelightName);
    }

    /**
     * Updates the robot's orientation in the Limelight for MegaTag2 pose estimation.
     * Should be called every frame with the latest robot orientation.
     *
     * @param robotYawDegrees The robot's current yaw angle in degrees.
     */
    public void updateRobotOrientation(double robotYawDegrees) {
        LimelightHelpers.SetRobotOrientation(limelightName, robotYawDegrees, 0, 0, 0, 0, 0);
    }

    /**
     * Sets the valid fiducial IDs for the Limelight to focus on specific tags (e.g., the speaker).
     *
     * @param validIDs An array of valid tag IDs to include.
     */
    public void setValidFiducialIDs(int[] validIDs) {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validIDs);
    }

    /**
     * Calculates the distance to the 2024 season's speaker using Limelight's MegaTag2 pose estimation.
     * Adjusts for alliance color by mirroring the tag's position if on the red alliance.
     *
     * @return The distance to the speaker in meters. Returns -1.0 if no tag is detected.
     */
    public double getDistanceToSpeaker() {
    // Retrieve the latest pose estimate using MegaTag2 (field-relative, blue alliance origin)

    
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

    // Check if the pose estimate is valid and if any tags are detected
    if (mt2 != null && mt2.tagCount > 0) {
        // Get the robot's position in the field (X, Y)
        Pose2d robotPose = mt2.pose;
        Translation2d robotTranslation = robotPose.getTranslation();

        // Get the alliance color from the DriverStation
        Alliance alliance = DriverStation.getAlliance().get();

        // Field dimensions (in meters)
        double fieldLength = Constants.FieldConstants.fieldLength; // Replace with actual field length in meters

        // Known speaker tag position in blue alliance coordinates (replace with actual values)
        double speakerXBlue = Constants.FieldConstants.SPEAKER_X_BLUE; // Speaker's X position in meters
        double speakerYBlue = Constants.FieldConstants.SPEAKER_Y_BLUE; // Speaker's Y position in meters

        double speakerX, speakerY;

        // Adjust speaker positions based on alliance color
        if (alliance == Alliance.Red) {
            // For the red alliance, mirror the speaker's X-coordinate across the field length
            speakerX = fieldLength - speakerXBlue;
            speakerY = speakerYBlue;
        } else {
            // For the blue alliance, use the speaker positions directly
            speakerX = speakerXBlue;
            speakerY = speakerYBlue;
        }

        Translation2d speakerTranslation = new Translation2d(speakerX, speakerY);

        // Calculate the distance between the robot and the speaker
        double distanceToSpeakerMeters = robotTranslation.getDistance(speakerTranslation);

        // Return the calculated distance
        return distanceToSpeakerMeters;
    } else {
        // No tags detected
        System.out.println("Warning: No MegaTag2 tags detected.");
        return -1.0;
    }
}

}
