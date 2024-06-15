package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.AllianceFlipUtil;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.vision.ApriltagVision;

public class CommandRotateToPose extends Command {

  private final CommandSwerveDrivetrain swerve;
  private Rotation2d desiredAngle;
  SwerveRequest.FieldCentric drive;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));

  public CommandRotateToPose(CommandSwerveDrivetrain swerve, Rotation2d r) {
    this.swerve = swerve;
    this.desiredAngle = r;

    thetaController.setTolerance(Units.degreesToRadians(0.5));
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {    
    var currPose = swerve.getState().Pose;
    SmartDashboard.putNumber("desiredAngle", desiredAngle.getRadians());  
    thetaController.reset(currPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    var currPose = swerve.getState().Pose;
    SmartDashboard.putNumber("currPose", currPose.getRotation().getRadians());
    final double thetaVelocity = atGoal() ? 0.0 :
        thetaController.calculate(
            currPose.getRotation().getRadians(), desiredAngle.getRadians());
    SmartDashboard.putNumber("thetaVel", thetaVelocity);
    swerve.applyRequest(() -> drive.withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(thetaVelocity));
  }
  public boolean atGoal() {
    return thetaController.atGoal();
  }

  @Override
  public boolean isFinished(){
    return atGoal();
    // return true;
  }

  @Override
  public void end(boolean interrupted){
    Constants.Vision.autoAlignActice = false;
  }
}