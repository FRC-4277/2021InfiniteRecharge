package frc4277.galacticvision.pipelines;

import frc4277.galacticvision.controllers.MainApplication;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class ClosePipeline extends Pipeline {

  @Override
  public Mat processImage(MainApplication app, Mat mat, MainPipeline mainPipeline) {
    double closeWidth = app.getDouble(app.closeW);
    double closeHeight = app.getDouble(app.closeH);
    if (closeWidth == 0 || closeHeight == 0) {
      return mat;
    }
    Mat kernel =
        Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(closeWidth, closeHeight));
    Imgproc.morphologyEx(mat, mat, Imgproc.MORPH_CLOSE, kernel);
    return mat;
  }
}
