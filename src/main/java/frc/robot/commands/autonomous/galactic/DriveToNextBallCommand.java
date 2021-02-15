package frc.robot.commands.autonomous.galactic;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.GalacticSearch;

public class DriveToNextBallCommand extends CommandBase {
    private GalacticAutoCommand galacticAutoCommand;
    private DriveTrain driveTrain;
    private int ballIndex;

    /**
     * @param ballIndex 0-indexed (0, 1, or2)
     */
    public DriveToNextBallCommand(GalacticAutoCommand galacticAutoCommand, DriveTrain driveTrain, int ballIndex) {
        this.galacticAutoCommand = galacticAutoCommand;
        this.driveTrain = driveTrain;
        this.ballIndex = ballIndex;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        Pose2d currentPosition = driveTrain.getPose();
        Translation2d ballPosition = galacticAutoCommand.getPathDetected().getThreePowerCells().get(ballIndex);

        Translation2d a = currentPosition.getTranslation();
        Translation2d b = ballPosition;

        // See math at https://math.stackexchange.com/a/2109383
        double distance = a.getDistance(b);
        double xOfTargetPosition = a.getX() -
                ((GalacticSearch.DESIRED_DISTANCE_TO_BALL * (a.getX() - b.getX())) / distance);

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
