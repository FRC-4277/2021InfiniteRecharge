/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheel;

public class PositionWheelCommand extends CommandBase {
  private ColorWheel colorWheel;
  private boolean finished = false;
  private ColorWheel.WheelColor targetColor = null;
  private Boolean clockwise = null;

  /**
   * Creates a new PositionWheelCommand.
   */
  public PositionWheelCommand(ColorWheel colorWheel) {
    this.colorWheel = colorWheel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(colorWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.finished = false;
    this.targetColor = null;
    this.clockwise = null;
    colorWheel.resetFilter();
    colorWheel.setPositionStatus("init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (finished) {
      colorWheel.stopWheel();
      return;
    }

    colorWheel.setPositionStatus("???Waiting for saturation???");

    colorWheel.updateFilter();
    if (!colorWheel.isFilterSaturated()) {
      return;
    }

    colorWheel.setPositionStatus("???Waiting for FMS Color???");

    if (targetColor == null) {
      targetColor = colorWheel.getFMSTargetColor();
      if (targetColor == null) {
        System.out.println("Still don't know target color!");
        return;
      }
    }

    ColorWheel.WheelColor currentColor = colorWheel.getFilteredColor();
    
    if (currentColor == targetColor) {
      colorWheel.stopWheel();
      colorWheel.setPositionStatus("!!!!FINISHED!!!!");
      finished = true;
    } else {
      // Must keep spinning
      if (clockwise == null) {
        clockwise = colorWheel.shouldSpinClockwise(currentColor, targetColor);
      }
      colorWheel.setPositionStatus((clockwise ? "CW" : "CCW") + ", T = " + targetColor + ", C = " + currentColor);
      if (clockwise) {
        colorWheel.spinClockwise();
      } else {
        colorWheel.spinCounterclockwise();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    colorWheel.stopWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
