package frc4277.galacticvision.pipelines;

import frc4277.galacticvision.controllers.MainApplication;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class OpenPipeline extends Pipeline {

  @Override
  public Mat processImage(MainApplication app, Mat mat, MainPipeline mainPipeline) {
    double openWidth = app.getDouble(app.openW);
    double openHeight = app.getDouble(app.openH);
    if (openWidth == 0 || openHeight == 0) {
      return mat;
    }
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(openWidth, openHeight));
    Imgproc.morphologyEx(mat, mat, Imgproc.MORPH_OPEN, kernel);
    return mat;
  }
}
