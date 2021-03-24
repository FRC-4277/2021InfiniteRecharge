/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class GameTimer implements Sendable {

  private RobotContainer container;

  public GameTimer(RobotContainer container) {
    this.container = container;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("GameTimer");
    builder.addDoubleProperty("matchTime", Timer::getMatchTime, null);
    builder.addBooleanProperty("autonomous", container::isInAutonomous, null);
  }
}
