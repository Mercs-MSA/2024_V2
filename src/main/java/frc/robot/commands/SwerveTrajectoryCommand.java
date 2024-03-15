package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class SwerveTrajectoryCommand extends Command {
    private SwerveControllerCommand swerveControllerCommand;
    private Swerve s_Swerve;
    private Trajectory trajectory;
    private boolean firstCommand;
    private ProfiledPIDController thetaController;
    private final PIDController xController =
      new PIDController(4, Constants.Swerve.driveKI, Constants.Swerve.driveKD);
    private final PIDController yController =
      new PIDController(4, Constants.Swerve.driveKI, Constants.Swerve.driveKD);

    public SwerveTrajectoryCommand(Swerve s_Swerve, Trajectory trajectory, boolean firstCommand){
        this.trajectory = trajectory;
        this.s_Swerve = s_Swerve;
        this.firstCommand = firstCommand;

        this.thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }


    @Override
    public void initialize() {
        if (this.firstCommand){
            this.s_Swerve.resetOdometry(this.trajectory.getInitialPose());
        }
        this.swerveControllerCommand =
            new SwerveControllerCommand(
                this.trajectory,
                this.s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                xController,
                yController,
                this.thetaController,
                this.s_Swerve::setModuleStates,
                this.s_Swerve);
        this.swerveControllerCommand.schedule();
    }

    @Override
    public void end(boolean interrupted) {

    }
  
    @Override
    public void execute() {
    }
  
    @Override
    public boolean isFinished() {
        // return Constants.isPoseWithinTol(this.trajectory.getStates().get(this.trajectory.getStates().size()-1).poseMeters, 
        //                                 this.s_Swerve.getPose(), 
        //                                 Constants.AutoConstants.tol);
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atGoal();
    }
}