package frc.robot.util;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class CooperSendable implements Sendable {

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Cooper");
  }
}
