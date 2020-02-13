package frc4277.vision.pipelines;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc4277.vision.Constants;
import frc4277.vision.Main;
import frc4277.vision.pipelines.setting.Setting;
import frc4277.vision.util.RectUtil;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.*;

public class ContourPipeline extends Pipeline {
    public List<MatOfPoint> foundContours = Collections.emptyList();
    private Setting<Boolean> drawContours = new Setting<>("drawContours", Boolean.class, false, BuiltInWidgets.kToggleSwitch);
    private Setting<Double> normalBoundingRectSolidityMin = new Setting<>("normalBoundingRectSolidityMin", Double.class, 0.06, BuiltInWidgets.kTextView);
    private Setting<Double> normalBoundingRectSolidityMax = new Setting<>("normalBoundingRectSolidityMax", Double.class, 0.2, BuiltInWidgets.kTextView);
    private Setting<Double> normalBoundingRectAspectRatioMin = new Setting<>("normalBoundingRectAspectRatioMin", Double.class, 0.0, BuiltInWidgets.kTextView);
    private Setting<Double> normalBoundingRectAspectRatioMax = new Setting<>("normalBoundingRectAspectRatioMax", Double.class, 2.9, BuiltInWidgets.kTextView);
    private Setting<Double> momentYRatioMin = new Setting<>("momentYRatioMin", Double.class, 0.45, BuiltInWidgets.kTextView);
    private Setting<Double> momentYRatioMax = new Setting<>("momentYRatioMax", Double.class, 999.0, BuiltInWidgets.kTextView);
    private NetworkTable resultsTable = Main.getInstance().getResultsTable();


    public ContourPipeline() {
        super("Contour");
    }

    @Override
    public void process(Mat mat, Context context) {
        Point matCenter = new Point(mat.width() / 2D, mat.height() / 2D);
        int matWidth = mat.width();
        int matHeight = mat.height();

        Imgproc.findContours(mat, foundContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_TC89_L1);
        boolean drawContours = this.drawContours.get();

        List<PotentialSmartTarget> potentialTargets = new ArrayList<>();

        for (int i = 0; i < foundContours.size(); i++) {
            MatOfPoint contour = foundContours.get(i);

            // Moments
            Moments p = Imgproc.moments(contour);
            int momentX = (int) (p.get_m10() / p.get_m00());
            int momentY = (int) (p.get_m01() / p.get_m00());
            if (drawContours) {
                Imgproc.circle(mat, new Point(momentX, momentY), 4, new Scalar(52, 122, 235));
            }

            Rect normalBoundingRect = Imgproc.boundingRect(contour);
            int normalBoundingRectCenterX = normalBoundingRect.x + (normalBoundingRect.width / 2);
            int normalBoundingRectCenterY = normalBoundingRect.y + (normalBoundingRect.height / 2);
            Point normalBoundingRectCenter = new Point(normalBoundingRectCenterX, normalBoundingRectCenterY);


            double momentXRatio = (momentX - normalBoundingRect.x) / (double) normalBoundingRect.width;
            double momentYRatio = (momentY - normalBoundingRect.y) / (double) normalBoundingRect.height;

            double area = Imgproc.contourArea(contour);
            float normalBoundingRectAspectRatio = (float) normalBoundingRect.width / normalBoundingRect.height;
            double normalBoundingRectSolidity = Imgproc.contourArea(contour)/(normalBoundingRect.width*normalBoundingRect.height);

            var minRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            Point[] minRectPoints = new Point[4];
            minRect.points(minRectPoints);
            RectUtil.RectanglePoints minRectPointsSorted = RectUtil.findCorners(minRectPoints, normalBoundingRectCenter);
            var centroid = minRectPointsSorted.center;

            // CHECK FOR PROPERTIES

            if (normalBoundingRectSolidity < normalBoundingRectSolidityMin.get()) {
                // Fail condition
                if (drawContours) {
                    Imgproc.putText(mat, "F sol min" + normalBoundingRectSolidity, normalBoundingRectCenter,
                            Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 255), 1);
                }
                continue;
            }
            if (normalBoundingRectSolidity > normalBoundingRectSolidityMax.get()) {
                // Fail condition
                if (drawContours) {
                    Imgproc.putText(mat, "F sol max" + normalBoundingRectSolidity, normalBoundingRectCenter,
                            Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 255), 1);
                }
                continue;
            }
            if (normalBoundingRectAspectRatio < normalBoundingRectAspectRatioMin.get()) {
                // Fail condition
                if (drawContours) {
                    Imgproc.putText(mat, "F a.ra min" + normalBoundingRectAspectRatio, normalBoundingRectCenter,
                            Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 255), 1);
                }
                continue;
            }
            if (normalBoundingRectAspectRatio > normalBoundingRectAspectRatioMax.get()) {
                // Fail condition
                if (drawContours) {
                    Imgproc.putText(mat, "F a.ra max" + normalBoundingRectSolidity, normalBoundingRectCenter,
                            Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 255), 1);
                }
                continue;
            }
            if (momentYRatio < momentYRatioMin.get()) {
                // Fail condition
                if (drawContours) {
                    Imgproc.putText(mat, "F mo.y min" + momentYRatio, normalBoundingRectCenter,
                            Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 255), 1);
                }
                continue;
            }
            if (momentYRatio > momentYRatioMax.get()) {
                // Fail condition
                if (drawContours) {
                    Imgproc.putText(mat, "F mo.y max" + normalBoundingRectSolidity, normalBoundingRectCenter,
                            Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 255), 1);
                }
                continue;
            }

            // END CHECK

            if (drawContours) {
                Imgproc.drawContours(mat, foundContours, i, new Scalar(255, 0, 0), -1);
                Imgproc.drawContours(mat, foundContours, i, new Scalar(255, 150, 0), 5);
            }

            // From Chameleon
            MatOfInt tempInt = new MatOfInt();
            MatOfPoint2f tempMat2f = new MatOfPoint2f();
            MatOfPoint tempMatOfPoint = new MatOfPoint();

            var convex = tempInt;
            tempMatOfPoint.fromList(contour.toList());
            Imgproc.convexHull(tempMatOfPoint, convex);
            var combinedList = contour.toList();

            Point[] contourArray = contour.toArray();
            Point[] hullPoints = new Point[convex.rows()];
            List<Integer> hullContourIdxList = convex.toList();
            for (int j = 0; j < hullContourIdxList.size(); j++) {
                hullPoints[j] = contourArray[hullContourIdxList.get(j)];
            }
            tempMat2f.fromArray(hullPoints);

            MatOfPoint2f polyOutput = new MatOfPoint2f(); //todo :reuse mats
            Imgproc.approxPolyDP(tempMat2f, polyOutput, 5, true);

            var polyList = polyOutput.toList();

            Comparator<Point> distanceProvider = Comparator.comparingDouble((Point point) -> Math.sqrt(Math.pow(centroid.x - point.x, 2) + Math.pow(centroid.y - point.y, 2)));

            try {

                // top left and top right are the poly corners closest to the bounding box (min rect) tl and tr
                Point tl = polyList.stream().min(Comparator.comparingDouble((Point p1) -> distanceBetween(p1, minRectPointsSorted.topLeft))).get();
                Point tr = polyList.stream().min(Comparator.comparingDouble((Point p1) -> distanceBetween(p1, minRectPointsSorted.topRight))).get();


                var bl = polyList.stream().filter(point -> point.x < centroid.x && point.y > centroid.y).max(distanceProvider).get();
                var br = polyList.stream().filter(point -> point.x > centroid.x && point.y > centroid.y).max(distanceProvider).get();

                RectUtil.RectanglePoints targetCorners = new RectUtil.RectanglePoints(tl, tr, bl, br);

                potentialTargets.add(
                        new PotentialSmartTarget(
                                targetCorners.center,
                                minRectPointsSorted,
                                targetCorners,
                                minRectPointsSorted.calculateArea(),
                                matCenter,
                                matWidth,
                                matHeight)
                );
                //return target2020ResultMat;
            } catch (NoSuchElementException e) {
                if (drawContours) {
                    Imgproc.putText(mat, "F nse" + normalBoundingRectSolidity, normalBoundingRectCenter,
                            Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 255), 1);
                }
                continue;
            }
        }

        Optional<PotentialSmartTarget> potentialTargetOptional = potentialTargets.stream().max(Comparator.comparingDouble(o -> o.minRectArea));
        if (potentialTargetOptional.isPresent()) {
            PotentialSmartTarget target = potentialTargetOptional.get();
            resultsTable.getEntry("x").setDouble(target.center.x);
            resultsTable.getEntry("y").setDouble(target.center.y);
            resultsTable.getEntry("degX").setDouble(target.degreesHorizontal);
            resultsTable.getEntry("degY").setDouble(target.degreesVertical);
            resultsTable.getEntry("minRectArea").setDouble(target.minRectArea);
        }
    }

    /**
     * Targets that we detect corners for
     */
    public static class PotentialSmartTarget {
        public Point center;
        public RectUtil.RectanglePoints minRect;
        public RectUtil.RectanglePoints targetCorners;
        public double minRectArea;
        public double degreesHorizontal;
        public double degreesVertical;

        public PotentialSmartTarget(Point center, RectUtil.RectanglePoints minRect, RectUtil.RectanglePoints targetCorners,
                                    double minRectArea, Point matCenter, int matWidth, int matHeight) {
            this.center = center;
            this.minRect = minRect;
            this.targetCorners = targetCorners;
            this.minRectArea = minRectArea;
            this.degreesHorizontal = ((center.x - matCenter.x) / matWidth) * Constants.PSEYE_HORIZONTAL_FOV;
            this.degreesVertical = ((center.x - matCenter.x) / matWidth) * Constants.PSEYE_VERTICAL_FOV;
        }
    }

    private double distanceBetween(Point a, Point b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }

    @Override
    public List<Setting<?>> getSettings() {
        return List.of(drawContours, normalBoundingRectSolidityMin, normalBoundingRectSolidityMax, normalBoundingRectAspectRatioMin,
                normalBoundingRectAspectRatioMax, momentYRatioMin, momentYRatioMax);
    }
}
