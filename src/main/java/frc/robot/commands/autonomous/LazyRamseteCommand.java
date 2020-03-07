package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;

import java.util.function.Supplier;

public class LazyRamseteCommand extends CommandBase {
    private DriveTrain driveTrain;
    private Supplier<Trajectory> trajectorySupplier;
    private RamseteCommand ramseteCommand;

    public LazyRamseteCommand(DriveTrain driveTrain, Supplier<Trajectory> trajectorySupplier) {
        this.driveTrain = driveTrain;
        this.trajectorySupplier = trajectorySupplier;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (ramseteCommand == null) {
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
        }
    }

    @Override
    public boolean isFinished() {
        return ramseteCommand != null && ramseteCommand.isFinished();
    }
}
