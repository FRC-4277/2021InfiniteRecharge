package frc.robot.commands.autonomous.galactic;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.GalacticSearch;

public class ZoomToMiddleCommand extends CommandBase {
    private GalacticAutoCommand galacticAutoCommand;
    private DriveTrain driveTrain;
    private RamseteCommand ramseteCommand;

    public ZoomToMiddleCommand(GalacticAutoCommand galacticAutoCommand, DriveTrain driveTrain) {
        this.galacticAutoCommand = galacticAutoCommand;
        this.driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = driveTrain.getPose();
        // Target is the same y-level, but at the end zone facing 0 degrees
        Pose2d target = new Pose2d(Units.feetToMeters(6*2.5), Units.feetToMeters(15 / 2), new Rotation2d());

        galacticAutoCommand.setMessage("[Zoom to Middle] Current pos: " + currentPose);
        galacticAutoCommand.setMessage("[Zoom to Middle] Target pos: " + target);
        // Add middle waypoint so it tries to take quickest path
        Trajectory trajectory = driveTrain.generateTrajectory(currentPose, target, 1.0, 0.5, false, false);
        ramseteCommand = driveTrain.generateRamseteCommand(trajectory);
        ramseteCommand.initialize();
    }

    @Override
    public void execute() {
        //galacticAutoCommand.setMessage("[Zoom to End] Moving...");
        ramseteCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        galacticAutoCommand.setMessage("[Zoom to Middle] End");
        driveTrain.stopDrive();
        if (ramseteCommand != null) {
            ramseteCommand.end(interrupted);
            ramseteCommand = null;
        }
    }

    @Override
    public boolean isFinished() {
        return ramseteCommand != null && ramseteCommand.isFinished();
    }
}
