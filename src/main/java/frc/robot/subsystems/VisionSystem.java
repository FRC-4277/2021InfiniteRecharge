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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.limelight.Limelight;
import frc.robot.util.limelight.Pipeline;
import frc.robot.util.limelight.StreamMode;
import frc.robot.util.limelight.Target;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import static frc.robot.Constants.Vision.Limelight.*;
import frc.robot.Constants.Vision.Pixy2Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;

public class VisionSystem extends SubsystemBase implements VerifiableSystem {
  private ShuffleboardTab driverTab;
  private ShuffleboardLayout layout;
  private Pipeline driverPipeline = new Pipeline("Driver", 0);
  private Pipeline portPipeline = new Pipeline("Port", 1);
  private Limelight limelight;

  private boolean calculateDistance = false;
  private double calculatedDistanceMeters = 0.0;
  private Pixy2 pixy2;
  private boolean usingPixy = false;
  private List<Block> blocksList;
  private Block largestBlock = null;

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

    this.pixy2 = Pixy2.createInstance(new SPILink());
    SmartDashboard.putNumber("Pixy2 Status", pixy2.init());
    pixy2.setLamp((byte) 0, (byte) 0);
    pixy2.setLED(200, 30, 255);
    this.usingPixy = true;
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

    if (usingPixy) {
      int blockCount = pixy2.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG_ALL, 25);
      SmartDashboard.putNumber("Block Count", blockCount);
      blocksList = pixy2.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2

      // Show blocks in SmartDashboard
      for (int i = 0; i < 5; i++) {
        SmartDashboard.putString("Block" + i, "");
      }
      for (int i = 0; i < blocksList.size(); i++) {
        Block block = blocksList.get(i);
        SmartDashboard.putString("Block" + i, block.toString());
      }

      // Find largest block
      largestBlock = null;
      for (Block block : blocksList) { // Loops through all blocks and finds the widest one
        if (largestBlock == null) {
          largestBlock = block;
        } else if (block.getWidth() > largestBlock.getWidth()) {
          largestBlock = block;
        }
      }
      if (largestBlock != null) {
        SmartDashboard.putString("X Value", Integer.toString(largestBlock.getX()));
        SmartDashboard.putString("BlockLargest", largestBlock.toString());
      } else {
        SmartDashboard.putString("X Value", "null");
      }
    }
  }

  public void setUsingPixy(boolean usingPixy) {
    this.usingPixy = usingPixy;
  }

  public List<Block> getBlocksList() {
    return blocksList;
  }

  public Optional<Block> getLargestBlock() {
    this.usingPixy = true;
    return Optional.ofNullable(largestBlock);
  }

  public Optional<Double> getBallTargetDegrees() {
    Optional<Block> blockOptional = getLargestBlock();
    if (blockOptional.isPresent()) {
      Block block = blockOptional.get();
      double x = block.getX();
      double xFromCenter = (x - (Pixy2Constants.MAX_X  / 2));
      return Optional.of((xFromCenter / Pixy2Constants.MAX_X) * Pixy2Constants.HORIZONTAL_FOV_DEG);
    } else {
      return Optional.empty();
    }
  }

  public double getCalculatedDistanceMeters() {
    return calculatedDistanceMeters;
  }

  public void setCalculateDistance(boolean calculateDistance) {
    this.calculateDistance = calculateDistance;
  }

  private void calculateDistanceIfNeeded() {
    if (!calculateDistance) {
      return;
    }
    Optional<Target> targetOptional = limelight.getTarget();

    if (targetOptional.isEmpty()) {
      targetOptional = limelight.getLastTarget();
    }

    if (targetOptional.isPresent()) {
      Target target = targetOptional.get();
      double yAngleDeg = target.getY();
      double yAngleRad = Math.toRadians(yAngleDeg);
      this.calculatedDistanceMeters = (PORT_CENTER_HEIGHT_M - MOUNT_HEIGHT_M) / Math.tan(MOUNT_ANGLE_RAD + yAngleRad);
    }
  }

  @Override
  public List<Verification> getVerifications(VerificationSystem system) {
    return null;
  }
}
