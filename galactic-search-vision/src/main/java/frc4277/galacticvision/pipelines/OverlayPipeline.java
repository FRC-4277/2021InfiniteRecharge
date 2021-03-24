package frc4277.galacticvision.pipelines;

import frc4277.galacticvision.controllers.MainApplication;
import java.util.ArrayList;
import java.util.List;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class OverlayPipeline extends Pipeline {
  @Override
  public Mat processImage(MainApplication app, Mat mat, MainPipeline mainPipeline) {
    Mat originalImage = mainPipeline.getNormalImage().clone();
    List<ResultContour> resultContours = mainPipeline.getResults();

    List<MatOfPoint> contours = new ArrayList<>();
    resultContours.forEach(resultContour -> contours.add(resultContour.getContour()));

    for (ResultContour resultContour : resultContours) {
      MatOfPoint2f contourPoly = resultContour.getContourPoly();

      // Draw circle
      Point center = new Point();
      float[] radius = new float[1];
      Imgproc.minEnclosingCircle(contourPoly, center, radius);
      Imgproc.circle(originalImage, center, (int) radius[0], new Scalar(0, 255, 0));

      // Draw (scalar = BGR)
      // Imgproc.drawContours(originalImage, contours, i, new Scalar(0, 255, 255), -1);
      Imgproc.circle(originalImage, center, 3, new Scalar(0, 0, 255), 2);
      // Imgproc.drawContours(originalImage, contours, i, new Scalar(255, 0, 0), 1);

      // Draw rectangle
      // Rect normalBoundingRect = Imgproc.boundingRect(contour);
      /*Imgproc.rectangle(originalImage, new Point(normalBoundingRect.x, normalBoundingRect.y),
      new Point(normalBoundingRect.x + normalBoundingRect.width, normalBoundingRect.y + normalBoundingRect.height),
      new Scalar(203, 192, 255), 1);*/
    }
    return originalImage;
  }
}
