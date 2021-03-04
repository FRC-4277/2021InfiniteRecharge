package frc4277.galacticvision.pipelines;

import frc4277.galacticvision.controllers.MainApplication;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class Dilation2Pipeline extends Pipeline{

    @Override
    public Mat processImage(MainApplication app, Mat mat, MainPipeline mainPipeline) {
        double dilationWidth = app.getDouble(app.dilation2W);
        double dilationHeight = app.getDouble(app.dilation2H);
        if (dilationWidth == 0 || dilationHeight == 0) {
            return mat;
        }
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilationWidth, dilationHeight));
        Imgproc.dilate(mat, mat, kernel);
        return mat;
    }
}
