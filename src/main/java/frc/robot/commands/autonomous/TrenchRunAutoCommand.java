/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionSystem;

public class TrenchRunAutoCommand extends CommandBase {
  //private static final double MAXIMUM_DISTANCE_RIGHT_M = 
  //private DriveTrain driveTrain;
  private VisionSystem visionSystem;
  private boolean finished = false;

  /**
   * Creates a new TrenchRunAutoCommand.
   */
  public TrenchRunAutoCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(driveTrain, visionSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
