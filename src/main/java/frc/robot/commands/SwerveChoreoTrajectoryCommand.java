package frc.robot.commands;

import java.util.Optional;

//intall using https://sleipnirgroup.github.io/ChoreoLib/dep/ChoreoLib.json
import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveChoreoTrajectoryCommand extends Command{
    private Swerve s_Swerve;
    private ChoreoTrajectory trajectory;

    public SwerveChoreoTrajectoryCommand(Swerve s_Swerve, ChoreoTrajectory trajectory){
        this.s_Swerve = s_Swerve;
        this.trajectory = trajectory;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        var thetaController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        Command swerveCommand = Choreo.choreoSwerveCommand(
            trajectory, // Choreo trajectory from above
            s_Swerve::getPose, // A function that returns the current field-relative pose of the robot: your
                                // wheel or vision odometry
            new PIDController(4, 0.0, 0.0), // PIDController for field-relative X
                                                                                    // translation (input: X error in meters,
                                                                                    // output: m/s).
            new PIDController(4, 0.0, 0.0), // PIDController for field-relative Y
                                                                                    // translation (input: Y error in meters,
                                                                                    // output: m/s).
            thetaController, // PID constants to correct for rotation
                            // error
            s_Swerve::driveRobotRelative, // needs to be robot-relative
            () -> {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
            }, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
            s_Swerve // The subsystem(s) to require, typically your drive subsystem only
        );
        swerveCommand.schedule();
    }
  
    @Override
    public void end(boolean interrupted) {

    }
  
    @Override
    public void execute() {
    }
  
    @Override
    public boolean isFinished() {
        return Constants.isPoseWithinTol(trajectory.getFinalPose(), s_Swerve.getPose(), Constants.AutoConstants.tol);
    }
}