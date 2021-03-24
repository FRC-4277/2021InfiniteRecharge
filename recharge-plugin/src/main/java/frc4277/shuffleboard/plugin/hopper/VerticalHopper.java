package frc4277.shuffleboard.plugin.hopper;

import edu.wpi.first.shuffleboard.api.data.ComplexData;
import java.util.Arrays;
import java.util.Map;
import java.util.Objects;
import java.util.StringJoiner;

public class VerticalHopper extends ComplexData<VerticalHopper> {
  public static final int MAXIMUM_CELLS = 5;
  public static final int MAXIMUM_INDEX = MAXIMUM_CELLS - 1; // 5 power cells (0..4)
  private boolean gateClosed;
  private boolean[] cellsPresent;
  private double speedRunning;

  /**
   * Represents the state of the VerticalHopper
   *
   * @param gateClosed Whether or not the solenoid gate that regulates the 0-th power cell (top
   *     most) is closed
   * @param cellsPresent Whether or not individual power cells are present (via sensors)
   * @param speedRunning Whether or not the belt is moving the power cells (positive indicates up
   *     towards shooter)
   */
  public VerticalHopper(boolean gateClosed, boolean[] cellsPresent, double speedRunning) {
    this.gateClosed = gateClosed;
    this.cellsPresent = cellsPresent;
    if (cellsPresent == null) {
      // Logic for if given cellsPresent is null
      cellsPresent = new boolean[MAXIMUM_CELLS];
      for (int i = 0; i <= MAXIMUM_INDEX; i++) {
        cellsPresent[i] = false;
      }
    } else if (cellsPresent.length < MAXIMUM_CELLS) {
      // Logic for if given cellsPresent is not full-length array
      boolean[] newCellsPresent = new boolean[MAXIMUM_CELLS];
      for (int i = 0; i <= MAXIMUM_INDEX; i++) {
        if (i <= (cellsPresent.length - 1)) {
          newCellsPresent[i] = cellsPresent[i];
        } else {
          newCellsPresent[i] = false;
        }
      }
      this.cellsPresent = newCellsPresent;
    }
    this.speedRunning = speedRunning;
  }

  @Override
  public Map<String, Object> asMap() {
    return Map.of(
        "gateClosed", gateClosed,
        "cellsPresent", cellsPresent,
        "speedRunning", speedRunning);
  }

  public boolean isGateClosed() {
    return gateClosed;
  }

  public boolean[] getCellsPresent() {
    return cellsPresent;
  }

  public boolean isCellPresent(int index) {
    if (index > MAXIMUM_INDEX) {
      throw new IllegalArgumentException(
          "Maximum index is " + MAXIMUM_INDEX + ", was given " + index);
    }
    return cellsPresent[index];
  }

  public double getSpeedRunning() {
    return speedRunning;
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    VerticalHopper that = (VerticalHopper) o;
    return gateClosed == that.gateClosed
        && Double.compare(that.speedRunning, speedRunning) == 0
        && Arrays.equals(cellsPresent, that.cellsPresent);
  }

  @Override
  public int hashCode() {
    int result = Objects.hash(gateClosed, speedRunning);
    result = 31 * result + Arrays.hashCode(cellsPresent);
    return result;
  }

  @Override
  public String toString() {
    return new StringJoiner(", ", VerticalHopper.class.getSimpleName() + "[", "]")
        .add("gateClosed=" + gateClosed)
        .add("cellsPresent=" + Arrays.toString(cellsPresent))
        .add("speedRunning=" + speedRunning)
        .toString();
  }
}
