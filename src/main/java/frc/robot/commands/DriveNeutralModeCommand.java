package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

public class DriveNeutralModeCommand extends InstantCommand {
    public DriveNeutralModeCommand(DriveTrain driveTrain, NeutralMode neutralMode) {
        super(() -> driveTrain.setNeutralMode(neutralMode), driveTrain);
    }
}
