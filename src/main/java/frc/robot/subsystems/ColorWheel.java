/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;

import java.util.*;
import java.util.function.Function;

import static frc.robot.Constants.ColorWheel.*;

public class ColorWheel extends SubsystemBase {
  private static final int FILTER_SIZE = 5;
  private WPI_TalonSRX motor = new WPI_TalonSRX(MOTOR_ID);
  private ColorSensorV3 colorSensor = new ColorSensorV3(COLOR_SENSOR_PORT);
  private ColorMatch colorMatch = new ColorMatch();
  private ModeFilter modeFilter = new ModeFilter(FILTER_SIZE); // 5 * 20ms = maximum 100ms delay
  private ShuffleboardTab tab;
  private NetworkTableEntry positionStatusEntry, rotationStatusEntry;
  private Color lastColor;
  private ColorMatchResult lastResult;
  private int lastProximity;

  /**
   * Creates a new ColorWheel.
   * @param colorWheelTab
   */
  public ColorWheel(ShuffleboardTab colorWheelTab) {
    this.tab = colorWheelTab;
    motor.configFactoryDefault();
    motor.setInverted(MOTOR_INVERTED);
    motor.setNeutralMode(NeutralMode.Brake);

    colorMatch.setConfidenceThreshold(0.96); //todo : increase if needed

    for (WheelColor wheelColor : WheelColor.values()) {
      colorMatch.addColorMatch(wheelColor.getColor());
    }

    tab.add(new CalibrateWheelColorCommand(this)).withPosition(0, 0).withSize(2, 1);
    tab.add(new PositionWheelCommand(this)).withPosition(2, 0).withSize(2, 1);
    tab.add(new RotationWheelCommand(this)).withPosition(4, 0).withSize(2, 1);
    tab.add(new SpinWheelClockwiseCommand(this)).withPosition(0, 1).withSize(2, 1);
    tab.add(new SpinWheelCounterclockwiseCommand(this)).withPosition(2, 1).withSize(2, 1);

    positionStatusEntry = tab.add("Position Status", "N/A")
    .withWidget(BuiltInWidgets.kTextView)
    .withPosition(6, 0)
    .withSize(2, 1)
    .getEntry();
    rotationStatusEntry = tab.add("Rotation Status", "N/A")
    .withWidget(BuiltInWidgets.kTextView)
    .withPosition(6, 1)
    .withSize(2, 1)
    .getEntry();

    tab.addNumber("Proximity", () -> lastProximity)
    .withWidget(BuiltInWidgets.kTextView)
    .withPosition(6, 2)
    .withSize(1, 1);

    tab.add("Filter Size", FILTER_SIZE)
    .withWidget(BuiltInWidgets.kTextView)
    .withPosition(5, 2)
    .withSize(1, 1)
    .getEntry()
    .addListener((notification) -> {
      if (notification.value.isDouble()) {
        modeFilter.reset();
        modeFilter.size = Double.valueOf(notification.value.getDouble()).intValue();
      }
    }, EntryListenerFlags.kUpdate);

    tab.addString("Last Colors", () -> {
      List<WheelColor> lastColors = modeFilter.getLast();
      if (lastColors == null || lastColors.isEmpty()) {
        return "none";
      } else {
        List<String> colorStrings = new ArrayList<>();
        for (WheelColor color : lastColors) {
          if (color == null) {
            continue;
          }
          colorStrings.add(color.name());
        }
        return String.join(", ", colorStrings);
      }
    })
    .withPosition(4, 1)
    .withSize(2, 1);
    tab.addNumber("Detected Color R", () -> getColorValue((color) -> color.red)).withWidget(BuiltInWidgets.kTextView)
    .withPosition(0, 2)
    .withSize(1, 1);
    tab.addNumber("Detected Color G", () -> getColorValue((color) -> color.green)).withWidget(BuiltInWidgets.kTextView)
    .withPosition(1, 2)
    .withSize(1, 1);
    tab.addNumber("Detected Color B", () -> getColorValue((color) -> color.blue)).withWidget(BuiltInWidgets.kTextView)
    .withPosition(2, 2)
    .withSize(1, 1);
    tab.addString("Filtered Color", () ->  {
      WheelColor filtered = getFilteredColor();
      if (filtered == null) {
        return "N/A";
      } else {
        return filtered.name();
      }
    })
    .withPosition(3, 2)
    .withSize(1, 1);
    tab.addString("Confidence Level", () -> 
    (lastResult == null ? "N/A" : Double.toString(lastResult.confidence)))
    .withPosition(4, 2)
    .withSize(1, 1);
  }

  private double getColorValue(Function<Color, Double> colorFunction) {
    if (lastColor != null){
      return colorFunction.apply(lastColor);
    }
    return -1;
  }

  public int getProximity() {
    return lastProximity = colorSensor.getProximity();
  }

  public void setPositionStatus(String status) {
    positionStatusEntry.setString(status);
  }

  public void setRotationStatus(String status) {
    rotationStatusEntry.setString(status);
  }

  public Color getDetectedColor() {
    return colorSensor.getColor();
  }

  public WheelColor getFilteredColor() {
    return modeFilter.getMode();
  }

  public void resetFilter() {
    modeFilter.reset();
  }

  public void updateFilter() {
    Color detectedColor = lastColor = getDetectedColor();
    ColorMatchResult result = lastResult = colorMatch.matchColor(detectedColor);
    if (result == null) {
      return;
    }
    WheelColor wheelColor = WheelColor.fromResult(result);
    if (wheelColor == null) {
      return;
    }
    modeFilter.update(wheelColor);
  }

  public boolean isFilterSaturated() {
    return modeFilter.isSaturated();
  }

  private void spin(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }

  public void spinClockwise(double speed) {
    spin(Math.abs(speed));
  }

  public void spinClockwise() {
    spinClockwise(DEFAULT_SPEED);
  }

  public void spinCounterclockwise(double speed) {
    spin(-Math.abs(speed));
  }

  public void spinCounterclockwise() {
    spinCounterclockwise(DEFAULT_SPEED);
  }

  public void stopWheel() {
    spin(0.0);
  }

  public WheelColor getFMSTargetColor() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0)
    {
      switch (gameData.charAt(0))
      {
        case 'B': //todo: OFFSET COLORS DEPENDING ON POSITION
          return WheelColor.BLUE;
        case 'G':
          return WheelColor.GREEN;
        case 'R':
          return WheelColor.RED;
        case 'Y':
          return WheelColor.YELLOW;
        default:
          return null;
      }
    }
    return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Algorithm to figure out to go left or right
  public boolean shouldSpinClockwise(ColorWheel.WheelColor currentColor, ColorWheel.WheelColor targetColor) {
    List<WheelColor> colorsInClockwise =
            Arrays.asList(WheelColor.RED, WheelColor.YELLOW, WheelColor.BLUE, WheelColor.GREEN);
    int currentIndex = colorsInClockwise.indexOf(currentColor);
    int targetIndex = colorsInClockwise.indexOf(targetColor);

    // Next color clockwise
    int nextClockwiseIndex = currentIndex + 1;
    nextClockwiseIndex %= colorsInClockwise.size();

    boolean targetIsDirectlyClockwise = nextClockwiseIndex == targetIndex;

    return targetIsDirectlyClockwise;
  }

  public static class ModeFilter {
    private LinkedList<WheelColor> lastColors;
    private Map<WheelColor, Integer> frequencyMap;
    private int size;

    public ModeFilter(int size) {
      this.lastColors = new LinkedList<>();
      this.frequencyMap = new HashMap<>();
      this.size = size;
    }

    public void update(WheelColor wheelColor) {
      // Add to frequency map
      int currentCount = frequencyMap.getOrDefault(wheelColor, 0);
      currentCount++;
      frequencyMap.put(wheelColor, currentCount);
      // Add to list
      lastColors.add(wheelColor);

      // Remove first element if size is too big
      if (lastColors.size() > size) {
        WheelColor firstColor = lastColors.poll();
        int count = frequencyMap.getOrDefault(firstColor, 1);
        count--;
        frequencyMap.put(firstColor, count);
      }
    }

    public WheelColor getMode() {
      if (lastColors.isEmpty()) {
        return null;
      }
      return Collections.max(frequencyMap.entrySet(), Map.Entry.comparingByValue()).getKey();
    }

    public int getSize() {
      return lastColors.size();
    }

    public List<WheelColor> getLast() {
      return Collections.unmodifiableList(lastColors);
    }

    public void reset() {
      lastColors.clear();
      frequencyMap.clear();
    }

    public boolean isSaturated() {
      return lastColors.size() >= size;
    }
  }

  public enum WheelColor {
    BLUE(ColorMatch.makeColor(/*0.143, 0.427, 0.429*/0.126, 0.423, 0.449)),
    RED(ColorMatch.makeColor(/*0.561, 0.232, 0.114*/.497,.358,.144)),
    GREEN(ColorMatch.makeColor(/*0.197, 0.561, 0.240*/.172,.573,.253)),
    YELLOW(ColorMatch.makeColor(/*0.361, 0.524, 0.113*/.316,.560,.123));

    private static double EPSILON = 0.001;
    private Color color;

    WheelColor(Color color) {
      this.color = color;
    }

    public Color getColor() {
      return color;
    }

    public static WheelColor fromResult(ColorMatchResult result) {
      if (result == null) {
        return null;
      }
      Color color = result.color;
      
      // Attempt to find via reference, most likely works
      for (WheelColor wheelColor : WheelColor.values()) {
        if (wheelColor.color == color) {
          return wheelColor;
        }
      }

      // Attempt to find via RGB values
      double red = color.red;
      double green = color.green;
      double blue = color.blue;
      for (WheelColor wheelColor : WheelColor.values()) {
        Color wheelColorColor = wheelColor.getColor();
        if ((Math.abs(wheelColorColor.red - red) <= EPSILON) && (Math.abs(wheelColorColor.green - green) <= EPSILON) &&
        (Math.abs(wheelColorColor.blue - blue) <= EPSILON)) {
          return wheelColor;
        }
      }
      
      return null;
    }
  }
}
