package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HookElevator;


public class MoveHookDownCommand extends CommandBase {
    private final HookElevator hookElevator;

    public MoveHookDownCommand(HookElevator hookElevator) {
        this.hookElevator = hookElevator;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.hookElevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        hookElevator.moveDown();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        hookElevator.stop();
    }
}
