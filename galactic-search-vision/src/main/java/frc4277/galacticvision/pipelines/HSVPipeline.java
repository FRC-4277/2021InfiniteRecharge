package frc4277.galacticvision.pipelines;

import frc4277.galacticvision.controllers.MainApplication;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class HSVPipeline extends Pipeline {
    @Override
    public Mat processImage(MainApplication app, Mat mat, MainPipeline mainPipeline) {
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV, 3);
        double hMin = app.getDouble(app.hMin);
        double hMax = app.getDouble(app.hMax);
        double sMin = app.getDouble(app.sMin);
        double sMax = app.getDouble(app.sMax);
        double vMin = app.getDouble(app.vMin);
        double vMax = app.getDouble(app.vMax);

        Core.inRange(mat,
            new Scalar(hMin, sMin, vMin),
            new Scalar(hMax, sMax, vMax),
        mat);

        return mat;
    }
}
