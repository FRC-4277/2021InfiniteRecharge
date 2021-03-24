package frc.robot.subsystems;

import static org.junit.Assert.*;

import org.junit.Test;

public class DriveTrainTest {
  private static final double EPSILON = 0.1;

  @Test
  public void unitConversions() {
    // Rotations of hex shaft
    assertEquals(1, DriveTrain.ticksToRotations(21934.08), EPSILON);
    assertEquals(21934.08, DriveTrain.rotationsToTicks(1), EPSILON);
    assertEquals(0.2182589115, DriveTrain.ticksToMeters(9999), EPSILON);
    assertEquals(1007876.373, DriveTrain.metersToTicks(22), 20); // Higher epsilon for ticks
  }
}
