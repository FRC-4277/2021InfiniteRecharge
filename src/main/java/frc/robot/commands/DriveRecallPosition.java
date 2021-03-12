package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;

public class DriveRecallPosition extends CommandBase {
    private static final double PAUSE_SECONDS_AT_END = 0.25;
    private final DriveTrain driveTrain;
    private final int index;
    private RamseteCommand ramseteCommand;
    private boolean finished = false;
    private Timer endTimer;

    public DriveRecallPosition(DriveTrain driveTrain, int index) {
        this.driveTrain = driveTrain;
        this.index = index;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        finished = false;
        endTimer = null;

        Pose2d storedPosition = driveTrain.getStoredPosition(index);
        if (storedPosition == null) {
            finished = true;
            return;
        }

        // Figure out whether to use forward or backward trajectory, depending on whether they generate properly
        // , and which one is faster (if both generate properly)
        Trajectory forwardTrajectory = generateTrajectory(storedPosition, false);
        Trajectory backwardTrajectory = generateTrajectory(storedPosition, true);
        Trajectory trajectory;
        if (forwardTrajectory != null && backwardTrajectory != null) {
            // Find quicker trajectory
            trajectory = forwardTrajectory.getTotalTimeSeconds() < backwardTrajectory.getTotalTimeSeconds()
                    ? forwardTrajectory : backwardTrajectory;
        } else if (forwardTrajectory != null) {
            trajectory = forwardTrajectory;
        } else if (backwardTrajectory != null) {
            trajectory = backwardTrajectory;
        } else {
            // If all failed to generate, end command
            finished = true;
            return;
        }

        ramseteCommand = driveTrain.generateRamseteCommand(trajectory);

        // Use brake mode for driving
        driveTrain.setNeutralMode(NeutralMode.Brake);
    }

    private Trajectory generateTrajectory(Pose2d targetPosition, boolean reversed) {
        try {
            return driveTrain.generateTrajectory(
                    driveTrain.getPose(), targetPosition, false, reversed);
        } catch (Exception e) {
            DriverStation.reportError("Failed to generate trajectory to stored position." +
                            " (reversed=" + reversed + ")", e.getStackTrace());
        }
        return null;
    }

    @Override
    public void execute() {
        if (ramseteCommand != null) {
            // Drive with RAMSETE
            ramseteCommand.execute();

            // Start end timer if ramsete command has finished, and it hasn't been started yet
            if (ramseteCommand.isFinished() && endTimer == null) {
                endTimer = new Timer();
                endTimer.reset();
                endTimer.start();
            }

            // Set finished to true when end timer has ran for PAUSE_SECONDS_AT_END seconds
            if (endTimer != null) {
                finished |= endTimer.hasElapsed(PAUSE_SECONDS_AT_END);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stopDrive();
        // Go back to coast mode
        driveTrain.setNeutralMode(NeutralMode.Coast);
        if (ramseteCommand != null) {
            ramseteCommand.end(interrupted);
            ramseteCommand = null;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
