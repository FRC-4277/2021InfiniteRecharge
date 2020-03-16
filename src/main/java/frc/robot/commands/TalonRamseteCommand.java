package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

import java.util.function.BiConsumer;

import static frc.robot.Constants.DriveTrain.*;

public class TalonRamseteCommand extends RamseteCommand {
  public TalonRamseteCommand(Trajectory trajectory, DriveTrain driveTrain, boolean dependOnDrive) {
    super(
      trajectory,
      driveTrain::getPose,
      new RamseteController(kRamseteB, kRamseteZeta),
      KINEMATICS,
      new BiConsumer<>() {
        private double previousLeftVelocity, previousRightVelocity;
        private double previousTime;
        private boolean firstRun = true;
        private Timer timer = new Timer();

        @Override
        public void accept(Double leftMetersPerSecond, Double rightMetersPerSecond) {
          if (Robot.isSimulation()) {
            driveTrain.setSimWheelSpeeds(new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond));
          }
          if (firstRun) {
            DifferentialDriveWheelSpeeds wheelSpeeds = driveTrain.getWheelSpeeds();
            previousLeftVelocity = wheelSpeeds.leftMetersPerSecond;
            previousRightVelocity = wheelSpeeds.rightMetersPerSecond;

            timer.reset();
            timer.start();

            previousTime = timer.get();
            firstRun = false;
          }
          double currentTime = timer.get();

          double dt = currentTime - previousTime;
          boolean dtIsPositive = dt > 0;

          // FF Calculations
          double leftAcceleration = dtIsPositive ? (leftMetersPerSecond - previousLeftVelocity) / dt : 0;
          double leftFeedforwardVolts = driveTrain.calculateFFVolts(leftMetersPerSecond, leftAcceleration);
          double leftFeedforwardPercent = leftFeedforwardVolts / MAX_BATTERY_V;
          // Velocity Calculations
          double leftTicksPerSecond = driveTrain.metersToTicks(leftMetersPerSecond);
          double leftTicksPerDs = leftTicksPerSecond / 10;
          // Drive Left
          if (!HAS_ENCODERS) {
            driveTrain.driveLeftSide(leftFeedforwardPercent);
          } else {
            driveTrain.driveLeftSideVelocity(leftTicksPerDs, leftFeedforwardPercent);
          }

          // FF Calculations
          double rightAcceleration = dtIsPositive ? (rightMetersPerSecond - previousRightVelocity) / dt : 0;
          double rightFeedforwardVolts = driveTrain.calculateFFVolts(rightMetersPerSecond, rightAcceleration);
          double rightFeedforwardPercent = rightFeedforwardVolts / MAX_BATTERY_V;
          // Velocity Calculations
          double rightTicksPerSecond = driveTrain.metersToTicks(rightMetersPerSecond);
          double rightTicksPerDs = rightTicksPerSecond / 10;
          // Drive Right
          if (!HAS_ENCODERS) {
            driveTrain.driveRightSide(rightFeedforwardPercent);
          } else {
            driveTrain.driveRightSideVelocity(rightTicksPerDs, rightFeedforwardPercent);
          }

          if (Robot.isSimulation()) {
            System.out.println("L: " + leftMetersPerSecond + " R:" + rightMetersPerSecond);
          }

          previousLeftVelocity = leftMetersPerSecond;
          previousRightVelocity = rightMetersPerSecond;
          previousTime = currentTime;

          driveTrain.feedWatchdog(); //So watchdog won't kill it
        }
      },
      dependOnDrive ? new Subsystem[]{driveTrain} : new Subsystem[]{}
    );
  }
}
