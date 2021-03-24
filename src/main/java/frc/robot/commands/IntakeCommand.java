/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VerticalHopper;

public class IntakeCommand extends CommandBase {
  private Intake intake;

  @SuppressWarnings("unused") // TODO: Figure out if I needed/wanted this
  private VerticalHopper verticalHopper;

  private static int intakeMinStopTimeMs = 400;
  private boolean ballIntaking = false;
  private Long ballIntakeTime = 0L;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(Intake intake, VerticalHopper verticalHopper) {
    this.intake = intake;
    this.verticalHopper = verticalHopper;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.ballIntaking = false;
    this.ballIntakeTime = 0L;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
