package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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

        //System.out.println("Stored position was " + storedPosition);
        //System.out.println("Current position: " + driveTrain.getPose());

        // Figure out whether to use forward or backward trajectory, depending on whether they generate properly
        // , and which one is faster (if both generate properly)
        Trajectory forwardTrajectory = generateTrajectory(storedPosition, false);
        Trajectory backwardTrajectory = generateTrajectory(storedPosition, true);
        Trajectory trajectory;
        boolean forwards;
        if (forwardTrajectory != null && backwardTrajectory != null) {
            // Find quicker trajectory
            forwards = forwardTrajectory.getTotalTimeSeconds() < backwardTrajectory.getTotalTimeSeconds();
            trajectory = forwards ? forwardTrajectory : backwardTrajectory;
            //System.out.println("USING DETECTED TRAJECTORY: " + (forwardTrajectory.getTotalTimeSeconds() < backwardTrajectory.getTotalTimeSeconds()));
        } else if (forwardTrajectory != null) {
            trajectory = forwardTrajectory;
            forwards = true;
            //System.out.println("Using forward");
        } else if (backwardTrajectory != null) {
            trajectory = backwardTrajectory;
            forwards = false;
            //System.out.println("Using backwards");
        } else {
            // If all failed to generate, end command
            finished = true;
            return;
        }

        ramseteCommand = driveTrain.generateRamseteCommand(trajectory, false);
        driveTrain.drawTrajectory(trajectory);
        ramseteCommand.initialize();

        System.out.println("=== RECALL === (" + index + ")");
        System.out.println("Current: " + driveTrain.getPose());
        System.out.println("Target: " + storedPosition);
        System.out.println("Direction: " + (forwards ? "Forward" : "Reverse"));
        System.out.println("Time: " + trajectory.getTotalTimeSeconds() + "s");
        System.out.println("===        ====");

        //System.out.println("Ramsete trajectory time: " + trajectory.getTotalTimeSeconds());

        // Use brake mode for driving
        driveTrain.setNeutralMode(NeutralMode.Brake);
    }

    private Trajectory generateTrajectory(Pose2d targetPosition, boolean reversed) {
        try {
            /*return driveTrain.generateTrajectory(
                    driveTrain.getPose(), targetPosition, 2.0, 0.5, false, reversed);*/
            return driveTrain.generateTrajectory(
                    driveTrain.getPose(), targetPosition, false, reversed);
        } catch (Exception e) {
            System.out.println("Failed to generate trajectory to stored position." +
                            " (reversed=" + reversed + ")");
            e.printStackTrace();
        }
        return null;
    }

    @Override
    public void execute() {
        if (ramseteCommand != null) {
            // Drive with RAMSETE
            ramseteCommand.execute();
            System.out.println("Executing");

            // Start end timer if ramsete command has finished, and it hasn't been started yet
            if (ramseteCommand.isFinished() && endTimer == null) {
                endTimer = new Timer();
                endTimer.reset();
                endTimer.start();
                System.out.println("End timer started");
            }

            // Set finished to true when end timer has ran for PAUSE_SECONDS_AT_END seconds
            if (endTimer != null && endTimer.hasElapsed(PAUSE_SECONDS_AT_END)) {
                finished = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stopDrive();
        System.out.println("END");
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
