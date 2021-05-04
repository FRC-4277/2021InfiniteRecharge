/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VerticalHopper;

public class AutoHopperMoveInCommand extends CommandBase {
  private final VerticalHopper hopper;
  private boolean pulsing = false;
  private Long lastBallInTime = null;
  private boolean waitingBetween = false;
  private Long startOfWaitTime = null;

  /** Creates a new AutoHopperMoveInCommand. */
  public AutoHopperMoveInCommand(VerticalHopper hopper) {
    this.hopper = hopper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.pulsing = false;
    this.lastBallInTime = null;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean ballPresentTop = !hopper.topBallSensor.get();
    if (ballPresentTop) {
      hopper.stopMoving();
      return;
    }

    boolean ballPresent = hopper.isBallPresentAtBottom();
    if (ballPresent && !pulsing) {
      this.pulsing = true;
      this.lastBallInTime = System.currentTimeMillis();
    }

    if (ballPresent
        && pulsing
        && (System.currentTimeMillis() - this.lastBallInTime <= hopper.getIndexRunTimeMs())) {
      hopper.moveUp();
    } else {
      hopper.stopMoving();
      if (pulsing) {
        this.waitingBetween = true;
      }
    }

    if (waitingBetween && startOfWaitTime == null) {
      startOfWaitTime = System.currentTimeMillis();
    }

    if (waitingBetween
        && (startOfWaitTime != null
            && (System.currentTimeMillis() - startOfWaitTime >= hopper.getIndexBetweenBallMs()))) {
      this.pulsing = false;
      this.lastBallInTime = null;
      this.waitingBetween = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
