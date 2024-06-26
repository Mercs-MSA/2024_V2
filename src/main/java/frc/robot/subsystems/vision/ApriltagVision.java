package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.List;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ApriltagVision extends SubsystemBase {

    private String cameraName = "FR";
    private PhotonCamera camera;
    private PhotonPipelineResult aprilTagResult;
    private boolean aprilTagHasTargets;
    private int fiducialID;
    private double aprilTagX, aprilTagY, aprilTagZAngle, aprilTagZ, distance;
    public static double yaw;
    private double pitch = -1;
    private PhotonTrackedTarget specificTags;
    

    public ApriltagVision(String cameraName) {
        this.cameraName = cameraName;
        camera = new PhotonCamera(cameraName);
        PhotonCamera.setVersionCheckEnabled(false);
        camera.setDriverMode(false);
        aprilTagResult = new PhotonPipelineResult();
        aprilTagHasTargets = false; 
    }

    @Override 
    public void periodic(){
        if (this.camera != null){
            aprilTagResult = camera.getLatestResult();
            
            aprilTagHasTargets = aprilTagResult.hasTargets();


            if (aprilTagHasTargets) {

                for (int i = 0; i < aprilTagResult.getTargets().size(); i++){
                    if (aprilTagResult.targets.get(i).getFiducialId() == 7 || aprilTagResult.targets.get(i).getFiducialId() == 4){
                        specificTags = aprilTagResult.targets.get(i);
                        break;
                    }
                }

                if (specificTags != null){
                    fiducialID = specificTags.getFiducialId();
                    aprilTagX = specificTags.getBestCameraToTarget().getX();
                    aprilTagY = specificTags.getBestCameraToTarget().getY();
                    aprilTagZ = specificTags.getBestCameraToTarget().getZ();
                    aprilTagZAngle = specificTags.getBestCameraToTarget().getRotation().getAngle();
                    yaw = specificTags.getYaw();
                    pitch = specificTags.getPitch();
                }
   
            }
            else {
                yaw = -1;
            } 


            if (aprilTagHasTargets){
                // Update SmartDashboard for AprilTag
                if (fiducialID == 7){
                    aprilTagZAngle = 180 - Units.radiansToDegrees(aprilTagZAngle);
                }
                if (fiducialID == 4){
                     aprilTagZAngle = 180 - aprilTagZAngle;
                }
                distance = PhotonUtils.calculateDistanceToTargetMeters(Units.inchesToMeters(8.75), 1.45, Units.degreesToRadians(30), pitch);
                SmartDashboard.putNumber(cameraName + " Fiducial ID", fiducialID);
                SmartDashboard.putNumber(cameraName + " AprilTag X (m)", aprilTagX);
                SmartDashboard.putNumber(cameraName + " AprilTag Y (m)", aprilTagY);
                SmartDashboard.putNumber(cameraName + " AprilTag Z (m)", aprilTagZ);
                SmartDashboard.putNumber(cameraName + " AprilTag Z Angle", aprilTagZAngle);
                SmartDashboard.putNumber(cameraName + " AprilTag Distance", Math.sqrt((aprilTagX*aprilTagX) + (aprilTagY*aprilTagY)));
                SmartDashboard.putNumber(cameraName + " AprilTag Yaw", yaw);
                SmartDashboard.putNumber(cameraName + " AprilTag Pitch", pitch);
            }
        }
    }

    public double getTimestampSeconds(){
        return this.aprilTagResult.getTimestampSeconds();
    }

    public boolean hasTargets(){
        return this.aprilTagHasTargets;
    }

    public boolean hasMultiTagEstimatedPose(){
        if (this.aprilTagResult != null){
            return this.aprilTagResult.getMultiTagResult().estimatedPose.isPresent;
        }
        return false;
    }

    /**
     * Gets the Fiducial ID of the AprilTag.
     * @return The Fiducial ID.
     */
    public int getFiducialID(){
        return fiducialID;
    }

    /**
     * Gets the X coordinate of the AprilTag in meters.
     * @return The X coordinate.
     */
    public double getAprilTagX(){
        return aprilTagX;
    }

    /**
     * Gets the X coordinate of the AprilTag in meters.
     * @return The X coordinate.
     */
    public static double getYaw(){
        return yaw;
    }

    /**
     * Gets the Y coordinate of the AprilTag in meters.
     * @return The Y coordinate.
     */
    public double getAprilTagY(){
        return aprilTagY;
    }

    /**
     * Gets the Z coordinate of the AprilTag in meters.
     * @return The Z coordinate.
     */
    public double getAprilTagZ(){
        return aprilTagZ;
    }

    /**
     * Gets the Z angle of the AprilTag in degrees.
     * @return The Z angle.
     */
    public double getAprilTagZAngle(){
        return aprilTagZAngle;
    }
}