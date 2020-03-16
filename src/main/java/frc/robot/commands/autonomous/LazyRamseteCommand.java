package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import java.util.function.Supplier;

public class LazyRamseteCommand extends CommandBase {
    private DriveTrain driveTrain;
    private Supplier<Trajectory> trajectorySupplier;
    private Command ramseteCommand;
    private boolean needsToGenerate = false;
    private boolean stopMovementAtEnd;

    public LazyRamseteCommand(DriveTrain driveTrain, Supplier<Trajectory> trajectorySupplier, boolean stopMovementAtEnd) {
        this.driveTrain = driveTrain;
        this.trajectorySupplier = trajectorySupplier;
        this.stopMovementAtEnd = stopMovementAtEnd;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        this.needsToGenerate = true;
    }

    @Override
    public void execute() {
        if (needsToGenerate) {
            this.needsToGenerate = false;
            ramseteCommand = driveTrain.generateRamseteCommand(trajectorySupplier.get(), false);
            ramseteCommand.initialize();
            return;
        }
        ramseteCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (ramseteCommand != null) {
            ramseteCommand.end(interrupted);
            if (stopMovementAtEnd) {
                driveTrain.stopDrive();
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (needsToGenerate) {
            return false;
        }
        return ramseteCommand != null && ramseteCommand.isFinished();
    }
}
