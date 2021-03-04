package frc.robot.commands.autonomous.galacticvideo;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

import java.util.function.Supplier;

public class GalacticResetOdometryCommand extends InstantCommand {
    public GalacticResetOdometryCommand(GalacticAutoVideoCommand galacticAutoVideoCommand, DriveTrain driveTrain, Supplier<Pose2d> poseSupplier) {
        super(() -> {
            Pose2d pose = poseSupplier.get();
            driveTrain.resetOdometry(poseSupplier.get());
            galacticAutoVideoCommand.setMessage("[Reset Odometry] Pose reset to " + pose);
        }, driveTrain);
    }
}
