package frc4277.galacticvision.pipelines;

import frc4277.galacticvision.controllers.MainApplication;
import org.opencv.core.Mat;

public abstract class Pipeline {
    public Pipeline() {
    }

    public abstract Mat processImage(MainApplication app, Mat mat, MainPipeline mainPipeline);
}
