package frc4277.shuffleboard.plugin;

import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableMap;
import edu.wpi.first.shuffleboard.api.data.DataType;
import edu.wpi.first.shuffleboard.api.plugin.Description;
import edu.wpi.first.shuffleboard.api.plugin.Plugin;
import edu.wpi.first.shuffleboard.api.widget.ComponentType;
import edu.wpi.first.shuffleboard.api.widget.WidgetType;
import frc4277.shuffleboard.plugin.hopper.VerticalHopperType;
import frc4277.shuffleboard.plugin.hopper.VerticalHopperWidget;

import java.util.List;
import java.util.Map;

@SuppressWarnings("rawtypes")
@Description(
    group = "frc4277.shuffleboard.plugin",
    name = "Infinite Recharge",
    version = "0.0.1",
    summary = "Add widgets for various subsystems for 2020 Competition Robot Code"
)
public class InfiniteRechargePlugin extends Plugin {
    @Override
    public List<DataType> getDataTypes() {
        return ImmutableList.of(
                VerticalHopperType.INSTANCE
        );
    }

    @Override
    public List<ComponentType> getComponents() {
        return ImmutableList.of(
                WidgetType.forAnnotatedWidget(VerticalHopperWidget.class)
        );
    }

    @Override
    public Map<DataType, ComponentType> getDefaultComponents() {
        return ImmutableMap.<DataType, ComponentType>builder()
                .put(VerticalHopperType.INSTANCE, WidgetType.forAnnotatedWidget(VerticalHopperWidget.class))
                .build();
    }
}
