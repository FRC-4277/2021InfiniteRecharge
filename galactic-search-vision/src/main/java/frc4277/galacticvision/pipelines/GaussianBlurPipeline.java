package frc4277.galacticvision.pipelines;

import frc4277.galacticvision.controllers.MainApplication;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class GaussianBlurPipeline extends Pipeline {

    @Override
    public Mat processImage(MainApplication app, Mat mat, MainPipeline mainPipeline) {
        double blurWidth = app.getDouble(app.gaussianBlurW);
        double blurHeight = app.getDouble(app.gaussianBlurH);
        if (blurWidth == 0 || blurHeight == 0) {
            return mat;
        }

        Imgproc.GaussianBlur(mat, mat, new Size(blurWidth, blurHeight), 0);

        return mat;
    }
}
