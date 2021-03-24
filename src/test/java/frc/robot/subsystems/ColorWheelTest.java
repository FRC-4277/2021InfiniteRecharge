package frc.robot.subsystems;

import static org.junit.Assert.*;

import org.junit.Test;

public class ColorWheelTest {

  @Test
  public void filterAllSame() {
    ColorWheel.ModeFilter greenModeFilter = new ColorWheel.ModeFilter(5);
    for (int i = 0; i < 5; i++) {
      greenModeFilter.update(ColorWheel.WheelColor.GREEN);
    }
    assertEquals("Expected Green", greenModeFilter.getMode(), ColorWheel.WheelColor.GREEN);

    ColorWheel.ModeFilter redModeFilter = new ColorWheel.ModeFilter(6);
    for (int i = 0; i < 6; i++) {
      redModeFilter.update(ColorWheel.WheelColor.RED);
    }
    assertEquals("Expected Red", redModeFilter.getMode(), ColorWheel.WheelColor.RED);
  }

  @Test
  public void filterThreeSame() {
    ColorWheel.ModeFilter greenModeFilter = new ColorWheel.ModeFilter(5);
    greenModeFilter.update(ColorWheel.WheelColor.GREEN);
    greenModeFilter.update(ColorWheel.WheelColor.GREEN);
    greenModeFilter.update(ColorWheel.WheelColor.GREEN);
    greenModeFilter.update(ColorWheel.WheelColor.RED);
    greenModeFilter.update(ColorWheel.WheelColor.RED);
    assertEquals("Expected Green", greenModeFilter.getMode(), ColorWheel.WheelColor.GREEN);

    ColorWheel.ModeFilter redModeFilter = new ColorWheel.ModeFilter(5);
    redModeFilter.update(ColorWheel.WheelColor.RED);
    redModeFilter.update(ColorWheel.WheelColor.RED);
    redModeFilter.update(ColorWheel.WheelColor.RED);
    redModeFilter.update(ColorWheel.WheelColor.BLUE);
    redModeFilter.update(ColorWheel.WheelColor.YELLOW);
    assertEquals("Expected Red", greenModeFilter.getMode(), ColorWheel.WheelColor.GREEN);
  }

  @Test
  public void emptyCheck() {
    ColorWheel.ModeFilter modeFilter = new ColorWheel.ModeFilter(10);
    assertNull("Expected null", modeFilter.getMode());
  }

  @Test
  public void checkList() {
    ColorWheel.ModeFilter modeFilter = new ColorWheel.ModeFilter(6);
    modeFilter.update(ColorWheel.WheelColor.RED);
    modeFilter.update(ColorWheel.WheelColor.YELLOW);
    modeFilter.update(ColorWheel.WheelColor.BLUE);
    modeFilter.update(ColorWheel.WheelColor.GREEN);
    modeFilter.update(ColorWheel.WheelColor.RED);
    modeFilter.update(ColorWheel.WheelColor.YELLOW);
    modeFilter.update(ColorWheel.WheelColor.GREEN);
    assertEquals(modeFilter.getLast().get(0), ColorWheel.WheelColor.YELLOW);
    assertEquals(modeFilter.getLast().get(1), ColorWheel.WheelColor.BLUE);
    assertEquals(modeFilter.getLast().get(2), ColorWheel.WheelColor.GREEN);
    assertEquals(modeFilter.getLast().get(3), ColorWheel.WheelColor.RED);
    assertEquals(modeFilter.getLast().get(4), ColorWheel.WheelColor.YELLOW);
    assertEquals(modeFilter.getLast().get(5), ColorWheel.WheelColor.GREEN);
  }

  @Test
  public void overSizeCheck() {
    ColorWheel.ModeFilter modeFilter = new ColorWheel.ModeFilter(5);
    modeFilter.update(ColorWheel.WheelColor.RED);
    for (int i = 0; i < 5; i++) {
      modeFilter.update(ColorWheel.WheelColor.GREEN);
    }
    assertFalse(
        "Expected list not to contain RED anymore",
        modeFilter.getLast().contains(ColorWheel.WheelColor.RED));
  }

  @Test
  public void filterLoopCheck() {
    ColorWheel.ModeFilter modeFilter = new ColorWheel.ModeFilter(5);
    modeFilter.update(ColorWheel.WheelColor.GREEN);
    modeFilter.update(ColorWheel.WheelColor.YELLOW);
    modeFilter.update(ColorWheel.WheelColor.RED);
    modeFilter.update(ColorWheel.WheelColor.BLUE);
    modeFilter.update(ColorWheel.WheelColor.RED);
    // Start removing elements
    modeFilter.update(ColorWheel.WheelColor.RED);

    assertEquals("Expected size to stay at 5", 5, modeFilter.getLast().size());
    assertEquals("Expected mode to be Red", ColorWheel.WheelColor.RED, modeFilter.getMode());
  }
}
