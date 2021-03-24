package frc4277.galacticvision.pipelines;

import frc4277.galacticvision.controllers.MainApplication;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class BlurPipeline extends Pipeline {

  @Override
  public Mat processImage(MainApplication app, Mat mat, MainPipeline mainPipeline) {
    double blurWidth = app.getDouble(app.blurW);
    double blurHeight = app.getDouble(app.blurH);
    if (blurWidth == 0 || blurHeight == 0) {
      return mat;
    }
    Imgproc.blur(mat, mat, new Size(blurWidth, blurHeight));
    return mat;
  }
}
