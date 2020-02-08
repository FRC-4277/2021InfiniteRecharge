/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Gate.*;

public class Gate extends SubsystemBase {
  private Solenoid solenoid = new Solenoid(SOLENOID_ID);

  /**
   * Creates a new Gate.
   */
  public Gate() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggle() {
    if (solenoid.get() != CLOSE_STATE) {
      close();
    } else {
      open();
    }
  }

  public void close() {
    solenoid.set(CLOSE_STATE);
  }

  public void open() {
    solenoid.set(OPEN_STATE);
  }
}
