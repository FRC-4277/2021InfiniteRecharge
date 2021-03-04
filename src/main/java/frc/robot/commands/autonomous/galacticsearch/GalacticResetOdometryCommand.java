package frc.robot.commands.autonomous.galacticsearch;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

import java.util.function.Supplier;

public class GalacticResetOdometryCommand extends InstantCommand {
    public GalacticResetOdometryCommand(GalacticAutoCommand galacticAutoCommand, DriveTrain driveTrain, Supplier<Pose2d> poseSupplier) {
        super(() -> {
            Pose2d pose = poseSupplier.get();
            driveTrain.resetOdometry(poseSupplier.get());
            galacticAutoCommand.setMessage("[Reset Odometry] Pose reset to " + pose);
        }, driveTrain);
    }
}
