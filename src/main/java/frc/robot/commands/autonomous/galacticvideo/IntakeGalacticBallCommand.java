package frc.robot.commands.autonomous.galacticvideo;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.subsystems.vision.VisionSystem;
import frc.robot.Constants.GalacticSearchVideo;

import java.util.Optional;

public class IntakeGalacticBallCommand extends CommandBase {
    private static final double TURN_P = 0.025d;
    private static final double TURN_DEG_TOLERANCE = 5;
    private static final double MIN_TURN_ADJUST = 0.05;

    private GalacticAutoVideoCommand galacticAutoVideoCommand;
    private DriveTrain driveTrain;
    private VisionSystem visionSystem;
    private VerticalHopper verticalHopper;

    private Intake intake;
    private static int intakeMinStopTimeMs = 400; // From IntakeCommand.java
    private boolean ballIntaking = false; //
    private Long ballIntakeTime = 0L; //

    private Timer timer;

    public IntakeGalacticBallCommand(GalacticAutoVideoCommand galacticAutoVideoCommand, DriveTrain driveTrain, VisionSystem visionSystem,
                                     VerticalHopper verticalHopper, Intake intake) {
        this.galacticAutoVideoCommand = galacticAutoVideoCommand;
        this.driveTrain = driveTrain;
        this.visionSystem = visionSystem;
        this.verticalHopper = verticalHopper;
        this.intake = intake;
        // NOTE: Purposely not adding vertical hopper so AutoHopperMoveInCommands still runs...
        addRequirements(driveTrain, visionSystem);
    }

    @Override
    public void initialize() {
        visionSystem.setUsingPixy(true);
        timer = null;

        // From IntakeCommand.java BELOW:
        this.ballIntaking = false;
        this.ballIntakeTime = 0L;
        galacticAutoVideoCommand.setMessage("[Intake] Initialized");
    }

    @Override
    public void execute() {
        // Similar code to IntakeLineUpCommand!!
        if (timer == null) {
            Optional<Double> targetOptional = visionSystem.getBallTargetDegrees();
            double headingError = targetOptional.orElse(0.0);
            boolean withinTolerance = Math.abs(headingError) <= TURN_DEG_TOLERANCE;
            double steerAdjust = headingError * TURN_P;
            if (!withinTolerance && Math.abs(steerAdjust) < MIN_TURN_ADJUST) {
                steerAdjust = Math.copySign(MIN_TURN_ADJUST, steerAdjust);
            }
            //

            double speed = GalacticSearchVideo.DRIVE_TO_BALL_FOR_INTAKE_SPEED;
            driveTrain.rawTankDrive(speed + steerAdjust, speed - steerAdjust);

            if (verticalHopper.isBallPresentAtBottom() && timer == null) {
                timer = new Timer();
                timer.reset();
                timer.start();
                galacticAutoVideoCommand.setMessage("[Intake] Detected ball at bottom, done soon!");
            }
            galacticAutoVideoCommand.setMessage(String.format("[Intake] Heading error %.2f, speed %.2f, steer adjust %.2f",
                    headingError, speed, steerAdjust));
        }

        // From IntakeCommand.java BELOW:
        boolean intakeBallPresent = !intake.intakeSensor.get();
        if (intakeBallPresent && !ballIntaking) {
            ballIntaking = true;
        }
        if (ballIntaking && (System.currentTimeMillis() - ballIntakeTime >= intakeMinStopTimeMs)) {
            ballIntaking = false;
        }
        if (!ballIntaking) {
            intake.runIntake();
        } else {
            intake.stopIntake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        visionSystem.setUsingPixy(false);
        intake.stopIntake(); // From IntakeCommand.java
        driveTrain.stopDrive();
    }

    @Override
    public boolean isFinished() {
        return timer != null && timer.hasElapsed(GalacticSearchVideo.WAIT_AFTER_INTAKE_SECONDS);
    }
}
