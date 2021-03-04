package frc.robot.commands.autonomous.galacticvideo;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.Vector;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.GalacticSearchVideo;

public class DriveToNextBallCommand extends CommandBase {
    private GalacticAutoVideoCommand galacticAutoVideoCommand;
    private DriveTrain driveTrain;
    private int ballIndex;
    private RamseteCommand ramseteCommand;

    /**
     * @param ballIndex 0-indexed (0, 1, or 2)
     */
    public DriveToNextBallCommand(GalacticAutoVideoCommand galacticAutoVideoCommand, DriveTrain driveTrain, int ballIndex) {
        this.galacticAutoVideoCommand = galacticAutoVideoCommand;
        this.driveTrain = driveTrain;
        this.ballIndex = ballIndex;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        Pose2d currentPosition = driveTrain.getPose();
        Translation2d ballPosition = galacticAutoVideoCommand.getPathDetected().getThreePowerCells().get(ballIndex);

        galacticAutoVideoCommand.setMessage("[Next Ball] Current pos: " + currentPosition);
        galacticAutoVideoCommand.setMessage("[Next Ball] Ball pos: " + ballPosition);

        Translation2d a = currentPosition.getTranslation();
        @SuppressWarnings("UnnecessaryLocalVariable") // Variable name for clarity
        Translation2d b = ballPosition;

        /*
        Goal is to find the point between the current position (A) and ball position (B) that
        is a certain distance from ball position (B)

        - Find vector between A & B
        - Find magnitude of vector
        - Multiply vector by (magnitude - desired distance to ball) / magnitude
        - Add vector to A to get the desired point (essentially getting end point of new vector)
         */
        double x = b.getX() - a.getX();
        double y = b.getY() - a.getY();
        Vector<N2> vector = VecBuilder.fill(x, y);
        double magnitude = a.getDistance(b);
        double desiredMagnitude = magnitude - GalacticSearchVideo.DESIRED_DISTANCE_TO_BALL;
        vector = vector.times(desiredMagnitude / magnitude);
        // Add vector to A
        double newX = a.getX() + vector.get(0, 0);
        double newY = a.getY() + vector.get(1, 0);
        // Have rotation be direction from A to B
        Pose2d targetPosition = new Pose2d(newX, newY, new Rotation2d(x, y));

        galacticAutoVideoCommand.setMessage("[Next Ball] Target pos: " + targetPosition);

        Trajectory trajectory = driveTrain.generateTrajectory(currentPosition, targetPosition, 1, 0.5, false, false);
        ramseteCommand = driveTrain.generateRamseteCommand(trajectory, false);
        ramseteCommand.initialize();
    }

    @Override
    public void execute() {
        //galacticAutoCommand.setMessage("[Next Ball] Moving...");
        ramseteCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        galacticAutoVideoCommand.setMessage("[Next Ball] End");
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
