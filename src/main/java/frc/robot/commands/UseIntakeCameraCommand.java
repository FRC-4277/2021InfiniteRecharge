package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSystem;


public class UseIntakeCameraCommand extends CommandBase {
    private final CameraSystem cameraSystem;

    public UseIntakeCameraCommand(CameraSystem cameraSystem) {
        this.cameraSystem = cameraSystem;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.cameraSystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        cameraSystem.switchToIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
