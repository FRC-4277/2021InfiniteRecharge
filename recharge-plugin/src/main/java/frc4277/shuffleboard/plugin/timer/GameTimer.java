package frc4277.shuffleboard.plugin.timer;

import edu.wpi.first.shuffleboard.api.data.ComplexData;

import java.util.Map;
import java.util.Objects;
import java.util.StringJoiner;

public class GameTimer extends ComplexData<GameTimer> {
    private double matchTime;
    private boolean autonomous;

    public GameTimer(double matchTime, boolean autonomous) {
        this.matchTime = matchTime;
        this.autonomous = autonomous;
    }

    public double getMatchTime() {
        return matchTime;
    }

    public void setMatchTime(double matchTime) {
        this.matchTime = matchTime;
    }

    public boolean isAutonomous() {
        return autonomous;
    }

    public void setAutonomous(boolean autonomous) {
        this.autonomous = autonomous;
    }

    @Override
    public Map<String, Object> asMap() {
        return Map.of(
            "matchTime", matchTime,
            "autonomous", autonomous);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        GameTimer gameTimer = (GameTimer) o;
        return Double.compare(gameTimer.matchTime, matchTime) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(matchTime);
    }

    @Override
    public String toString() {
        return new StringJoiner(", ", GameTimer.class.getSimpleName() + "[", "]")
                .add("matchTime=" + matchTime)
                .toString();
    }
}
