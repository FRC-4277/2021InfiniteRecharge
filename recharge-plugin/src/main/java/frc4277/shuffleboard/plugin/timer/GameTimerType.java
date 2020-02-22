package frc4277.shuffleboard.plugin.timer;

import edu.wpi.first.shuffleboard.api.data.ComplexDataType;
import edu.wpi.first.shuffleboard.api.util.Maps;

import java.util.Map;
import java.util.function.Function;

public class GameTimerType extends ComplexDataType<GameTimer> {
    public static final GameTimerType INSTANCE = new GameTimerType();

    protected GameTimerType() {
        super("GameTimer", GameTimer.class);
    }

    @Override
    public Function<Map<String, Object>, GameTimer> fromMap() {
        return map -> new GameTimer(Maps.getOrDefault(map, "matchTime", 0.0));
    }

    @Override
    public GameTimer getDefaultValue() {
        return new GameTimer(0.0);
    }
}
