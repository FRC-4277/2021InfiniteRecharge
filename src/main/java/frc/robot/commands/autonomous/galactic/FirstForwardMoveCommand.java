package frc.robot.commands.autonomous.galactic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.GalacticSearch;

public class FirstForwardMoveCommand extends CommandBase {
    private final GalacticAutoCommand galacticAutoCommand;
    private final DriveTrain driveTrain;
    private RamseteCommand ramseteCommand;

    public FirstForwardMoveCommand(GalacticAutoCommand galacticAutoCommand, DriveTrain driveTrain) {
        this.galacticAutoCommand = galacticAutoCommand;
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        GalacticPath path = galacticAutoCommand.getPathDetected();
        double distanceForward = path.isPowerCellClose() ?
                GalacticSearch.CLOSE_BALL_DISTANCE : GalacticSearch.FAR_BALL_DISTANCE;
        distanceForward -= GalacticSearch.DESIRED_DISTANCE_TO_BALL;
        ramseteCommand = driveTrain
                .generateRamseteCommand(driveTrain.generateXTrajectory(distanceForward), false);
        ramseteCommand.initialize();
    }

    @Override
    public void execute() {
        ramseteCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stopDrive();
        if (ramseteCommand != null) {
            ramseteCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return ramseteCommand != null && ramseteCommand.isFinished();
    }
}
