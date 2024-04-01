package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.ApriltagVision;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    public static Rotation2d goodStraightGyro = new Rotation2d();
    private double multiplier;
    private ProfiledPIDController thetaController =
      new ProfiledPIDController(2.5, 0.01, 0.1, new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));

    private double thetaVelocity = 0.0;
    public static boolean ranInitForAutoAim = false;


    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;

        

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.thetaController.setTolerance(Units.degreesToRadians(0.5));
        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);

        if (Constants.AllianceFlipUtil.shouldFlip()){
            multiplier = -1;
        }
        else {
            multiplier = 1;
        }

        addRequirements(s_Swerve);
    }

    @Override
    public void execute() { 

        if (Constants.Vision.autoRunning == false){
            // /* Get Values, Deadband*/
            double translationVal = Constants.Vision.manualDriveInvert * multiplier * MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
            double strafeVal = Constants.Vision.manualDriveInvert * multiplier * MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
            double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

            thetaVelocity = (thetaController.calculate(Swerve.poseEstimator.getEstimatedPosition().getRotation().getRadians(), goodStraightGyro.getRadians()) / Math.PI) * 10;

            if (Constants.Vision.autoAimActive){
                goodStraightGyro = Rotation2d.fromDegrees(Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees() - ApriltagVision.getYaw());

                s_Swerve.drive(
                    new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                    thetaVelocity, 
                    !robotCentricSup.getAsBoolean(), 
                    false //was originally true
                );
            }
            else if (Constants.Vision.autoAimActive == false && rotationVal != 0){
                goodStraightGyro = Swerve.poseEstimator.getEstimatedPosition().getRotation();
                
                s_Swerve.drive(
                    new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                    rotationVal * Constants.Swerve.maxAngularVelocity, 
                    !robotCentricSup.getAsBoolean(), 
                    false //was originally true
                ); 
            }
            else if (rotationVal == 0){

                s_Swerve.drive(
                    new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                    thetaVelocity, 
                    !robotCentricSup.getAsBoolean(), 
                    false //was originally true
                );
            }
            else {
                goodStraightGyro = Swerve.poseEstimator.getEstimatedPosition().getRotation();

                s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true
            );
            }





        }
        else {
            goodStraightGyro = Swerve.poseEstimator.getEstimatedPosition().getRotation();
        }

        if (goodStraightGyro != null){
            SmartDashboard.putNumber("goodStraightGyro teleop swerve", goodStraightGyro.getRadians());
            SmartDashboard.putNumber("rotationVal teleop swerve", -(goodStraightGyro.getRadians() - Swerve.poseEstimator.getEstimatedPosition().getRotation().getRadians()));
            SmartDashboard.putNumber("auto aim angle", Rotation2d.fromDegrees(Swerve.poseEstimator.getEstimatedPosition().getRotation().getDegrees() - ApriltagVision.getYaw()).getDegrees());
        }
    }
}