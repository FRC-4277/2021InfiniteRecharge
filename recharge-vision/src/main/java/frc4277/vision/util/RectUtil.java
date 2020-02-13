package frc4277.vision.util;

import org.opencv.core.Point;

public class RectUtil {

    public static RectanglePoints findCorners(Point[] points, Point center) {
        if (points.length < 4) {
            throw new IllegalArgumentException("Not a rectangle");
        }
        Point topLeft = null, topRight = null, bottomLeft = null, bottomRight = null;
        for (Point point : points) {
            if (point.x < center.x && point.y < center.y) {
                topLeft = point;
            } else if (point.x > center.x && point.y < center.y) {
                topRight = point;
            } else if (point.x < center.x && point.y > center.y) {
                bottomLeft = point;
            } else if (point.x > center.x && point.y > center.y) {
                bottomRight = point;
            }
        }
        Point newCenter = center;
        if (topLeft != null && bottomRight != null) {
            newCenter = new Point((topLeft.x + bottomRight.x) / 2, (topLeft.y + bottomRight.y) / 2);
        }
        return new RectanglePoints(topLeft, topRight, bottomLeft, bottomRight, newCenter);
    }

    public static class RectanglePoints {
        public Point topLeft, topRight, bottomLeft, bottomRight;
        public Point center;

        public RectanglePoints(Point topLeft, Point topRight, Point bottomLeft, Point bottomRight, Point center) {
            this.topLeft = topLeft;
            this.topRight = topRight;
            this.bottomLeft = bottomLeft;
            this.bottomRight = bottomRight;
            this.center = center;
        }

        public RectanglePoints(Point topLeft, Point topRight, Point bottomLeft, Point bottomRight) {
            this.topLeft = topLeft;
            this.topRight = topRight;
            this.bottomLeft = bottomLeft;
            this.bottomRight = bottomRight;
            this.center = new Point((topLeft.x + bottomRight.x) / 2, (topLeft.y + bottomRight.y) / 2);
        }

        public double calculateArea() {
            double height = distanceBetween(topLeft, bottomLeft);
            double width = distanceBetween(topLeft, topRight);
            return height * width;
        }
    }

    private static double distanceBetween(Point a, Point b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }
}