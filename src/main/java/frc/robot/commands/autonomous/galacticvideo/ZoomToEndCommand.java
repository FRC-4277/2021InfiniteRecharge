package frc.robot.commands.autonomous.galacticvideo;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.GalacticSearchVideo;

public class ZoomToEndCommand extends CommandBase {
    private GalacticAutoVideoCommand galacticAutoVideoCommand;
    private DriveTrain driveTrain;
    private RamseteCommand ramseteCommand;

    public ZoomToEndCommand(GalacticAutoVideoCommand galacticAutoVideoCommand, DriveTrain driveTrain) {
        this.galacticAutoVideoCommand = galacticAutoVideoCommand;
        this.driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = driveTrain.getPose();
        // Target is the same y-level, but at the end zone facing 0 degrees
        Pose2d target = new Pose2d(GalacticSearchVideo.ROBOT_END_X, currentPose.getY(), new Rotation2d());

        galacticAutoVideoCommand.setMessage("[Zoom to End] Current pos: " + currentPose);
        galacticAutoVideoCommand.setMessage("[Zoom to End] Target pos: " + target);
        // Add middle waypoint so it tries to take quickest path
        Trajectory trajectory = driveTrain.generateTrajectory(currentPose, target, true);
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
        galacticAutoVideoCommand.setMessage("[Zoom to End] End");
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
