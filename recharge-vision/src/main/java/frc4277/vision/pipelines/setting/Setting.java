package frc4277.vision.pipelines.setting;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc4277.vision.Constants;
import frc4277.vision.pipelines.Pipeline;

public class Setting<T> {
  private static final long MAXIMUM_CACHE_MS = 1000;

  private String key;
  private T defaultValue;
  private NetworkTableEntry entry;
  private Class<? extends T> valueClass;
  private T value;
  private long lastUpdateTimestamp = -1;
  private String widgetType;

  public Setting(String key, Class<? extends T> valueClass, T defaultValue, String widgetType) {
    this.key = key;
    this.valueClass = valueClass;
    this.defaultValue = defaultValue;
    this.widgetType = widgetType;
  }

  public Setting(
      String key, Class<? extends T> valueClass, T defaultValue, BuiltInWidgets builtInWidget) {
    this(key, valueClass, defaultValue, builtInWidget.getWidgetName());
  }

  public String getKey() {
    return key;
  }

  public T getDefaultValue() {
    return defaultValue;
  }

  public void setupAutomaticEntry(
      Pipeline pipeline, NetworkTable pipelineTable, ShuffleboardTab visionTab) {
    if (widgetType == null) {
      // No widget
      this.entry = pipelineTable.getEntry(key);
    } else {
      // Widget
      this.entry =
          visionTab
              .add(pipeline.getName() + "_" + key, defaultValue)
              .withWidget(widgetType)
              .getEntry();
    }
    entry.addListener(
        notification -> {
          Object value = notification.value.getValue();

          if (value == null || !notification.value.isValid()) {
            this.value = defaultValue;
            entry.setValue(defaultValue);
          }

          setValue(value);
        },
        Constants.NT_UPDATE_FLAGS);
  }

  @SuppressWarnings("unchecked")
  private void setValue(Object o) {
    try {
      if (o instanceof Double && valueClass.equals(Integer.class)) {
        Setting.this.value = (T) (Object) ((Double) o).intValue();
      } else {
        Setting.this.value = (T) o;
      }
      lastUpdateTimestamp = System.currentTimeMillis();
    } catch (Exception e) {
      System.out.println(
          "Failed to cast value of entry "
              + entry.getName()
              + " to "
              + valueClass.getCanonicalName());
    }
  }

  public T get() {
    if ((System.currentTimeMillis() - lastUpdateTimestamp) >= MAXIMUM_CACHE_MS) {
      setValue(entry.getValue().getValue());
    }
    if (value == null) {
      value = defaultValue;
      entry.setValue(defaultValue);
    }
    return value;
  }
}
