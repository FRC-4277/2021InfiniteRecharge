package frc.robot.commands.autonomous.galactic;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

import java.util.function.Supplier;

public class GalacticResetOdometryCommand extends InstantCommand {
    public GalacticResetOdometryCommand(DriveTrain driveTrain, Supplier<Pose2d> poseSupplier) {
        super(() -> driveTrain.resetOdometry(poseSupplier.get()), driveTrain);
    }
}
