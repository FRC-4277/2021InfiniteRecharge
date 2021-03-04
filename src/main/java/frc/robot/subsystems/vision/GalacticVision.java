package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.*;

public class GalacticVision {
    private static final String TABLE_NAME = "galactic";
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
    private final NetworkTableEntry xEntry = table.getEntry("x");
    private final NetworkTableEntry yEntry = table.getEntry("y");
    private final NetworkTableEntry areaEntry = table.getEntry("area");

    public List<PowerCellTarget> getLargestPowerCells() {
        List<PowerCellTarget> list = new ArrayList<>();
        double[] xValues = xEntry.getDoubleArray(new double[]{});
        double[] yValues = yEntry.getDoubleArray(new double[]{});
        double[] areaValues = areaEntry.getDoubleArray(new double[]{});

        double count = Math.min(xValues.length, Math.max(yValues.length, areaValues.length));
        for (int i = 0; i < count; i++) {
            list.add(new PowerCellTarget(xValues[i], yValues[i], areaValues[i]));
        }

        // Sort ascending
        list.sort(Comparator.comparingDouble(PowerCellTarget::getArea));
        // Flip so that it's in descending order (most area first)
        Collections.reverse(list);

        // Limit list to three largest targets
        if (list.size() > 3) {
            // Top three range: [0..3) (0 is inclusive & 3 is exclusive so it's indexes 0,1,2)
            list = new ArrayList<>(list.subList(0, 3));
        }

        return Collections.unmodifiableList(list);
    }

    public static class PowerCellTarget {
        private final double x, y, area;

        public PowerCellTarget(double x, double y, double area) {
            this.x = x;
            this.y = y;
            this.area = area;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getArea() {
            return area;
        }

        @Override
        public String toString() {
            return "PowerCellTarget{" +
                    "x=" + x +
                    ", y=" + y +
                    ", area=" + area +
                    '}';
        }
    }
}
