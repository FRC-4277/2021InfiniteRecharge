package frc4277.galacticvision.pipelines;

import frc4277.galacticvision.controllers.MainApplication;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class ErosionPipeline extends Pipeline{

    @Override
    public Mat processImage(MainApplication app, Mat mat, MainPipeline mainPipeline) {
        double erosionWidth = app.getDouble(app.erosionW);
        double erosionHeight = app.getDouble(app.erosionH);
        if (erosionWidth == 0 || erosionHeight == 0) {
            return mat;
        }
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(erosionWidth, erosionHeight));
        Imgproc.erode(mat, mat, kernel);
        return mat;
    }
}
