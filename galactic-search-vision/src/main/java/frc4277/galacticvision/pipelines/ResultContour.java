package frc4277.galacticvision.pipelines;

import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;

public class ResultContour {
    private final double x, y, area;
    private final MatOfPoint contour;
    private final MatOfPoint2f contourPoly;

    public ResultContour(double x, double y, double area, MatOfPoint contour, MatOfPoint2f contourPoly) {
        this.x = x;
        this.y = y;
        this.area = area;
        this.contour = contour;
        this.contourPoly = contourPoly;
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

    public MatOfPoint getContour() {
        return contour;
    }

    public MatOfPoint2f getContourPoly() {
        return contourPoly;
    }

    @Override
    public String toString() {
        return String.format("{x=%.2f, y=%.2f, area=%.2f}", x, y, area);
    }
}
