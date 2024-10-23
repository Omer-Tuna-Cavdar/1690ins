package frc.robot.Subsytems;

import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Vision class to interact with the Limelight camera.
 * This class can be used across different subsystems.
 */
public class Vision {

    private final String limelightName;

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
}
