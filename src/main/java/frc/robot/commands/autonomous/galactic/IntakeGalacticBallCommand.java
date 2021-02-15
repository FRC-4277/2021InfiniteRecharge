package frc.robot.commands.autonomous.galactic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.subsystems.VisionSystem;
import frc.robot.Constants.GalacticSearch;

import java.util.Optional;

public class IntakeGalacticBallCommand extends CommandBase {
    private static final double TURN_P = 0.025d;
    private static final double TURN_DEG_TOLERANCE = 5;
    private static final double MIN_TURN_ADJUST = 0.05;

    private DriveTrain driveTrain;
    private VisionSystem visionSystem;
    private VerticalHopper verticalHopper;

    private Timer timer;

    public IntakeGalacticBallCommand(DriveTrain driveTrain, VisionSystem visionSystem, VerticalHopper verticalHopper) {
        this.driveTrain = driveTrain;
        this.visionSystem = visionSystem;
        this.verticalHopper = verticalHopper;
        // NOTE: Purposely not adding vertical hopper so AutoHopperMoveInCommands still runs...
        addRequirements(driveTrain, visionSystem);
    }

    @Override
    public void initialize() {
        visionSystem.setUsingPixy(true);
        timer = null;
    }

    @Override
    public void execute() {
        // Similar code to IntakeLineUpCommand!!
        Optional<Double> targetOptional = visionSystem.getBallTargetDegrees();
        double headingError = targetOptional.orElse(0.0);
        if (Math.abs(headingError) <= TURN_DEG_TOLERANCE) {
            return;
        }
        double steerAdjust = headingError * TURN_P;
        if (Math.abs(steerAdjust) < MIN_TURN_ADJUST) {
            steerAdjust = Math.copySign(MIN_TURN_ADJUST, steerAdjust);
        }
        //

        double speed = GalacticSearch.DRIVE_TO_BALL_FOR_INTAKE_SPEED;
        driveTrain.rawTankDrive(speed + steerAdjust, speed - steerAdjust);

        if (verticalHopper.isBallPresentAtBottom() && timer == null) {
            timer = new Timer();
            timer.reset();
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        visionSystem.setUsingPixy(false);
        driveTrain.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return timer != null && timer.hasElapsed(GalacticSearch.WAIT_AFTER_INTAKE_SECONDS);
    }
}
