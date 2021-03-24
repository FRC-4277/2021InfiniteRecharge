package frc4277.galacticvision.pipelines;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc4277.galacticvision.GalacticVision;
import frc4277.galacticvision.controllers.MainApplication;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class ContourPipeline extends Pipeline {
  private NetworkTable table;
  private NetworkTableEntry xEntry;
  private NetworkTableEntry yEntry;
  private NetworkTableEntry areaEntry;

  public ContourPipeline() {}

  @Override
  public Mat processImage(MainApplication app, Mat mat, MainPipeline mainPipeline) {
    List<MatOfPoint> contours = new ArrayList<>();
    Imgproc.findContours(
        mat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_TC89_L1);
    // System.out.println("Contours size: " + contours.size());

    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

    List<ResultContour> resultContours = new ArrayList<>();
    for (int i = 0; i < contours.size(); i++) {
      MatOfPoint contour = contours.get(i);

      // Moments
      // Moments p = Imgproc.moments(contour);
      // double momentX = (p.get_m10() / p.get_m00());
      // double momentY = (p.get_m01() / p.get_m00());

      // Draw circle
      MatOfPoint2f contourPoly = new MatOfPoint2f();
      Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), contourPoly, 3, true);
      Point center = new Point();
      float[] radius = new float[1];
      Imgproc.minEnclosingCircle(contourPoly, center, radius);
      Imgproc.circle(mat, center, (int) radius[0], new Scalar(0, 255, 0));

      // Draw (scalar = BGR)
      Imgproc.drawContours(mat, contours, i, new Scalar(0, 255, 255), -1);
      Imgproc.circle(mat, center, 3, new Scalar(0, 0, 255));
      Imgproc.drawContours(mat, contours, i, new Scalar(255, 0, 0), 1);

      // Draw rectangle
      Rect normalBoundingRect = Imgproc.boundingRect(contour);
      Imgproc.rectangle(
          mat,
          new Point(normalBoundingRect.x, normalBoundingRect.y),
          new Point(
              normalBoundingRect.x + normalBoundingRect.width,
              normalBoundingRect.y + normalBoundingRect.height),
          new Scalar(203, 192, 255),
          1);

      // Calculate area from circle
      double area = Math.PI * Math.pow(radius[0], 2);

      resultContours.add(new ResultContour(center.x, center.y, area, contour, contourPoly));
    }

    double[] xValues = new double[resultContours.size()];
    double[] yValues = new double[resultContours.size()];
    double[] areaValues = new double[resultContours.size()];

    // Sort from smallest to largest area (ascending)
    resultContours.sort(Comparator.comparingDouble(ResultContour::getArea));
    // Flip so it's in descending order
    Collections.reverse(resultContours);

    mainPipeline.setResults(resultContours);

    for (int i = 0; i < resultContours.size(); i++) {
      ResultContour resultContour = resultContours.get(i);
      xValues[i] = resultContour.getX();
      yValues[i] = resultContour.getY();
      areaValues[i] = resultContour.getArea();
    }

    if (table == null) {
      if (GalacticVision.getInstance().getNTInstance() == null
          || !GalacticVision.getInstance().getNTInstance().isConnected()) {
        return mat;
      }
      table = GalacticVision.getInstance().getNTInstance().getTable("galactic");
      xEntry = table.getEntry("x");
      yEntry = table.getEntry("y");
      areaEntry = table.getEntry("area");
    }
    xEntry.setDoubleArray(xValues);
    yEntry.setDoubleArray(yValues);
    areaEntry.setDoubleArray(areaValues);

    return mat;
  }
}
