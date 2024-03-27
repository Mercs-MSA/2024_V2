package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.List;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ApriltagVision extends SubsystemBase {

    private String cameraName = "BR";
    private PhotonCamera camera;
    private PhotonPipelineResult aprilTagResult;
    private boolean aprilTagHasTargets;
    private int fiducialID;
    private double aprilTagX, aprilTagY, aprilTagZAngle, aprilTagZ = -1;
    private List<PhotonTrackedTarget> specificTags;
    

    public ApriltagVision(String cameraName) {
        this.cameraName = cameraName;
        camera = new PhotonCamera(cameraName);
        aprilTagResult = new PhotonPipelineResult();
        aprilTagHasTargets = false;
    }

    @Override 
    public void periodic(){
        if (this.camera != null){
            aprilTagResult = camera.getLatestResult();
            
            aprilTagHasTargets = aprilTagResult.hasTargets();


            if (aprilTagHasTargets) {

                specificTags = aprilTagResult.getTargets().stream().filter(x -> (x.getFiducialId() == 4 || x.getFiducialId() == 7)).collect(Collectors.toList());

                if (specificTags.size() > 0) {
                    PhotonTrackedTarget target = specificTags.get(0);
                    fiducialID = target.getFiducialId();
                    aprilTagX = target.getBestCameraToTarget().getX();
                    aprilTagY = target.getBestCameraToTarget().getY();
                    aprilTagZ = target.getBestCameraToTarget().getZ();
                    aprilTagZAngle = target.getBestCameraToTarget().getRotation().getAngle();
                    // Now that we know the target we automatically win!
                }
            } 
            else {
                fiducialID = -1;
                aprilTagX = -1.0;
                aprilTagY = -1.0;
                aprilTagZ = -1.0;
                aprilTagZAngle = -1.0;
            }

            if (aprilTagHasTargets){
                // Update SmartDashboard for AprilTag
                SmartDashboard.putNumber(cameraName + " Fiducial ID", fiducialID);
                SmartDashboard.putNumber(cameraName + " AprilTag X (m)", aprilTagX);
                SmartDashboard.putNumber(cameraName + " AprilTag Y (m)", aprilTagY);
                SmartDashboard.putNumber(cameraName + " AprilTag Z (m)", aprilTagZ);
                SmartDashboard.putNumber(cameraName + " AprilTag Z Angle", aprilTagZAngle);
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
        return aprilTagZAngle * (180 / Math.PI);
    }
}