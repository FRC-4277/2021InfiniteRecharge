// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.sixball;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.LazyRamseteCommand;
import frc.robot.commands.autonomous.ResetOdometryCommand;
import frc.robot.commands.hopper.AutoHopperMoveInCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VerticalHopper;
import frc.robot.subsystems.vision.VisionSystem;

import java.util.List;

public class SixBallAutoNoShootCommand extends SequentialCommandGroup {
  private static final double BALL_Y_COORDINATE = 7.457;
  private static final Pose2d START_POSE = new Pose2d(new Translation2d(3.16, BALL_Y_COORDINATE), new Rotation2d());
  private static final Pose2d RIGHT_BEFORE_FIRST_BALL_POSE = new Pose2d(new Translation2d(4.9, BALL_Y_COORDINATE), new Rotation2d());
  private static final Pose2d RIGHT_RIGHT_BEFORE_FIRST_BALL_POSE = new Pose2d(new Translation2d(4.2, BALL_Y_COORDINATE), new Rotation2d());
  private static final Pose2d END_POSITION = new Pose2d(new Translation2d(5.351, 6.864), Rotation2d.fromDegrees(12.5));
  private static final double MAX_VELOCITY = 4;
  private static final double MAX_ACCEL = 3;
  private static final int DESIRED_RPM_1 = 2400; // RPM for first shots
  private static final int DESIRED_RPM_2 = 2190; // RPM for second shots
  private Timer timer = new Timer();

  public SixBallAutoNoShootCommand(DriveTrain driveTrain, VisionSystem visionSystem, Shooter shooter, VerticalHopper hopper, Intake intake) {
    addCommands(
      // Start timer
      new InstantCommand(() -> {
        timer = new Timer();
        timer.reset();
        timer.start();
      }),
      // Use brake mode
      new DriveNeutralModeCommand(driveTrain, NeutralMode.Brake),
      // Set start pose & reset odometry
      new ResetOdometryCommand(driveTrain, START_POSE), // Will reset NavX heading also
      // Align & shoot in parallel
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitCommand(0.25),
          new FastShootAndHopperCommand(shooter, hopper, visionSystem, false, DESIRED_RPM_1, false)
        ),
        new VisionAlignCommand(driveTrain, visionSystem, false, false, false, false).withTimeout(1.5)
      ).withTimeout(4.3),
      new PrintTimerCommand(timer, "Finish first align & shoot"),
      // Drive to near first ball from current position via trajectory
      new LazyRamseteCommand(driveTrain, () -> {
        // Add an interior waypoint to ensure the robot faces 0 degrees
        List<Translation2d> interiorWaypoints = List.of(RIGHT_RIGHT_BEFORE_FIRST_BALL_POSE.getTranslation());
        //List<Translation2d> interiorWaypoints = List.of();
        return driveTrain.generateTrajectory(driveTrain.getPose(), RIGHT_BEFORE_FIRST_BALL_POSE, interiorWaypoints,
          3.5, 2.5, false, 0,
          driveTrain.convertPercentToVelocity(PickupThreeCommand.INITIAL_SPEED - 0.075));
      }),
      new PrintTimerCommand(timer, "Finish drive to right before first ball"),
      // Pickup three balls
      // Also use old hopper move in command for SPEEEED
      new ParallelDeadlineGroup(
        new PickupThreeCommand(driveTrain, visionSystem, intake).withTimeout(5.5),
        new AutoHopperMoveInCommand(hopper)
      ),
      new PrintTimerCommand(timer, "Finish pickup three balls"),
      // Drive back to end position for last shots
      new LazyRamseteCommand(driveTrain, () -> driveTrain.generateTrajectory(driveTrain.getPose(), END_POSITION, 4, 2, false, true, 0, 0)),
      new PrintTimerCommand(timer, "Finish drive back to end position"),
      new VisionAlignCommand(driveTrain, visionSystem, false, false, false, false).withTimeout(1.5)
      // Align & shoot in parallel
      /*new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new FastShootAndHopperCommand(shooter, hopper, visionSystem, false, DESIRED_RPM_2, false)
        ),
        new VisionAlignCommand(driveTrain, visionSystem, false, false, false, false).withTimeout(1.0)
      )*/
      /*new PrintTimerCommand(timer, "COMPLETE AUTONOMOUS"),
      // Go back to user selected neutral mode (brake or coast)
      new InstantCommand(() -> driveTrain.setNeutralMode(driveTrain.neutralModeChooser.getSelected()))*/
    );
  }
}
