/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.limelight.Limelight;
import frc.robot.util.limelight.Pipeline;
import frc.robot.util.limelight.StreamMode;
import frc.robot.util.limelight.Target;
import static frc.robot.Constants.Vision.Limelight.*;

import java.util.Map;
import java.util.Optional;
import java.util.function.Function;

public class VisionSystem extends SubsystemBase {
  private ShuffleboardTab driverTab;
  private ShuffleboardLayout layout;
  private Pipeline driverPipeline = new Pipeline("Driver", 0);
  private Pipeline portPipeline = new Pipeline("Port", 1);
  private Limelight limelight;

  private boolean calculateDistance = false;
  private double calculatedDistanceMeters = 0.0;

  /**
   * Creates a new VisionSystem.
   * 
   */
  public VisionSystem(ShuffleboardTab driverTab) {
    this.driverTab = driverTab;
    this.limelight = new Limelight(driverPipeline, portPipeline);

    this.layout = this.driverTab.getLayout("Limelight", BuiltInLayouts.kGrid)
    .withSize(4, 1)
    .withPosition(6, 1)
    .withProperties(
      Map.of(
    "Label position", "TOP",
    "Number of columns", 5,
    "Number of rows", 1
      )
    );

    this.layout.addBoolean("Target", () -> this.limelight.getTarget().isPresent())
    .withWidget(BuiltInWidgets.kBooleanBox);
    this.layout.addString("X Angle", () -> this.getLimelightDisplayProperty(Target::getX))
    .withWidget(BuiltInWidgets.kTextView);
    this.layout.addString("Y Angle", () -> this.getLimelightDisplayProperty(Target::getY))
    .withWidget(BuiltInWidgets.kTextView);
    this.layout.addString("Distance", () -> (calculateDistance ? "T" : "F") + " " + String.format("%.2f m", this.calculatedDistanceMeters))
            .withWidget(BuiltInWidgets.kTextView);
    this.layout.addString("Distance I", () -> String.format("%.2f in", Units.metersToInches(this.calculatedDistanceMeters)))
            .withWidget(BuiltInWidgets.kTextView);

    useDriverPipeline();
  }

  public Limelight getLimelight() {
    return limelight;
  }

  public void usePortPipeline() {
    limelight.setPipeline(portPipeline);
    limelight.setStreamMode(StreamMode.PIP_MAIN);
  }

  public void useDriverPipeline() {
    limelight.setPipeline(driverPipeline);
    limelight.setStreamMode(StreamMode.PIP_SECONDARY);
  }

  private String getLimelightDisplayProperty(Function<Target, Double> function) {
    Optional<Target> optional = limelight.getTarget();
    if (optional.isEmpty()) {
      return "N/A";
    }
    return String.format("%.2f", optional.map(function).get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    calculateDistanceIfNeeded();
  }

  public void setCalculateDistance(boolean calculateDistance) {
    this.calculateDistance = calculateDistance;
  }

  private void calculateDistanceIfNeeded() {
    if (!calculateDistance) {
      return;
    }
    Optional<Target> targetOptional = limelight.getTarget();

    if (targetOptional.isPresent()) {
      Target target = targetOptional.get();
      double yAngleDeg = target.getY();
      double yAngleRad = Math.toRadians(yAngleDeg);
      this.calculatedDistanceMeters = (PORT_CENTER_HEIGHT_M - MOUNT_HEIGHT_M) / Math.tan(MOUNT_ANGLE_RAD + yAngleRad);
    }
  }
}
