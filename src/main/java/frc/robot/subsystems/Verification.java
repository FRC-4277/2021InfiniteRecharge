package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.function.Supplier;

public class Verification {
  private NetworkTableEntry widgetEntry;
  private String name;
  private Supplier<Boolean> checker;

  public Verification(String name, Supplier<Boolean> checker) {
    this.name = name;
    this.checker = checker;
  }

  public boolean check() {
    return checker.get();
  }

  public NetworkTableEntry getWidgetEntry() {
    return widgetEntry;
  }

  public void setWidgetEntry(NetworkTableEntry widgetEntry) {
    this.widgetEntry = widgetEntry;
  }

  public String getName() {
    return name;
  }
}
