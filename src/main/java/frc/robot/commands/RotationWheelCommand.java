/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheel;

public class RotationWheelCommand extends CommandBase {
  private static final int TARGET_COLOR_CHANGES = 14; // 3.5 rotations, each having 4 colors
  private ColorWheel colorWheel;

  private boolean finished = false;
  private ColorWheel.WheelColor lastColor = null;
  private int colorChanges = 0;

  /**
   * Creates a new SpinWheelClockwiseCommand.
   */
  public RotationWheelCommand(ColorWheel colorWheel) {
    this.colorWheel = colorWheel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(colorWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.finished = false;
    this.lastColor = null;
    this.colorChanges = 0;
    colorWheel.resetFilter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (finished) {
      colorWheel.stopWheel();
      return;
    }

    if (!colorWheel.isFilterSaturated()) {
      colorWheel.updateFilter();
      return;
    }
    ColorWheel.WheelColor color = colorWheel.getFilteredColor();
    if (color != lastColor) {
      colorChanges++;
      this.lastColor = color;
    }

    if (colorChanges < TARGET_COLOR_CHANGES) {
      colorWheel.spinClockwise();
    } else {
      // DONE!
      colorWheel.stopWheel();
      finished = true;
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
