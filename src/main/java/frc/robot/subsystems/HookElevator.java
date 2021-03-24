package frc.robot.subsystems;

import static frc.robot.Constants.HookElevator.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

public class HookElevator extends SubsystemBase implements VerifiableSystem {
  private static final double DEFAULT_SPEED = 0.75;
  private TalonSRX motor = new TalonSRX(MOTOR_ID);

  public HookElevator() {
    motor.configFactoryDefault();
    motor.setInverted(MOTOR_INVERTED);
    motor.setNeutralMode(NeutralMode.Brake);
  }

  public void moveUp() {
    moveUp(DEFAULT_SPEED);
  }

  public void moveUp(double speed) {
    speed = Math.abs(speed);
    motor.set(ControlMode.PercentOutput, speed);
  }

  public void moveDown() {
    moveDown(-DEFAULT_SPEED);
  }

  public void moveDown(double speed) {
    speed = -Math.abs(speed);
    motor.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public List<Verification> getVerifications(VerificationSystem system) {
    return null;
  }
}
