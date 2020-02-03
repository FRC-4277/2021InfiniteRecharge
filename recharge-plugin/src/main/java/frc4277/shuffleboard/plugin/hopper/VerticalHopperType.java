package frc4277.shuffleboard.plugin.hopper;

import edu.wpi.first.shuffleboard.api.data.ComplexDataType;
import edu.wpi.first.shuffleboard.api.util.Maps;

import java.util.Map;
import java.util.function.Function;

public class VerticalHopperType extends ComplexDataType<VerticalHopper> {
    public static final VerticalHopperType INSTANCE = new VerticalHopperType();
    private static final boolean[] EMPTY_CELLS_PRESENT = new boolean[]{false, false, false, false, false};

    private VerticalHopperType() {
        super("VerticalHopper", VerticalHopper.class);
    }

    @Override
    public Function<Map<String, Object>, VerticalHopper> fromMap() {
        return map -> new VerticalHopper(
                Maps.getOrDefault(map, "gateClosed", true),
                Maps.getOrDefault(map, "cellsPresent", EMPTY_CELLS_PRESENT),
                Maps.getOrDefault(map, "speedRunning", 0.0)
        );
    }

    @Override
    public VerticalHopper getDefaultValue() {
        return new VerticalHopper(true, EMPTY_CELLS_PRESENT, 0.0);
    }
}
