package frc4277.shuffleboard.plugin.cooper;

import edu.wpi.first.shuffleboard.api.data.ComplexDataType;
import java.util.Map;
import java.util.function.Function;

public class CooperType extends ComplexDataType<Cooper> {
  public static final CooperType INSTANCE = new CooperType();

  protected CooperType() {
    super("Cooper", Cooper.class);
  }

  @Override
  public Function<Map<String, Object>, Cooper> fromMap() {
    return map -> new Cooper();
  }

  @Override
  public Cooper getDefaultValue() {
    return new Cooper();
  }
}
