package frc4277.vision.pipelines.setting;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc4277.vision.Constants;

public class Setting<T> {
    private static final long MAXIMUM_CACHE_MS = 1000;

    private String key;
    private T defaultValue;
    private NetworkTableEntry entry;
    private Class<? extends T> valueClass;
    private T value;
    private long lastUpdateTimestamp = -1;

    public Setting(String key, Class<? extends T> valueClass, T defaultValue) {
        this.key = key;
        this.valueClass = valueClass;
        this.defaultValue = defaultValue;
    }

    public String getKey() {
        return key;
    }

    public T getDefaultValue() {
        return defaultValue;
    }

    public void setupAutomaticEntry(NetworkTableEntry entry) {
        this.entry = entry;
        entry.addListener(notification -> {
            Object value = notification.value.getValue();

            if (value == null || !notification.value.isValid()) {
                this.value = defaultValue;
                entry.setValue(defaultValue);
            }

            setValue(value);
        }, Constants.NT_UPDATE_FLAGS);
    }

    @SuppressWarnings("unchecked")
    private void setValue(Object o) {
        try {
            Setting.this.value = (T) o;
            lastUpdateTimestamp = System.currentTimeMillis();
        } catch (Exception e) {
            System.out.println("Failed to cast value of entry " + entry.getName() + " to " + valueClass.getCanonicalName());
        }
    }

    public T get() {
        if ((System.currentTimeMillis() - lastUpdateTimestamp) >= MAXIMUM_CACHE_MS) {
            setValue(entry.getValue().getValue());
        }
        return value;
    }
}
