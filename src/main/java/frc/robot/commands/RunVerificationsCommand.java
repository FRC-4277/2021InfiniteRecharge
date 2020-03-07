package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VerificationSystem;


public class RunVerificationsCommand extends CommandBase {
  private final VerificationSystem verificationSystem;
  private boolean firstRun;
  private Timer timer = new Timer();

  public RunVerificationsCommand(VerificationSystem verificationSystem) {
    this.verificationSystem = verificationSystem;
    // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.verificationSystem);
  }

  @Override
  public void initialize() {
    this.firstRun = true;
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (firstRun) {
      verificationSystem.runVerifications();
      this.firstRun = false;
      return;
    }
    // Run verifications every 3 seconds while command is active
    if (timer.hasElapsed(3)) {
      timer.reset();
      timer.start();
      verificationSystem.runVerifications();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {

  }
}
