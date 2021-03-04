package frc4277.galacticvision.pipelines;

import frc4277.galacticvision.controllers.MainApplication;
import frc4277.galacticvision.util.RollingDoubleAverage;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class StatsPipeline extends Pipeline {
    @Override
    public Mat processImage(MainApplication app, Mat mat, MainPipeline mainPipeline) {
        List<String> strings = new ArrayList<>();
        // === FPS
        double totalMs = 0;
        for (RollingDoubleAverage rollingDoubleAverage : mainPipeline.getAverageMap().values()) {
            totalMs += rollingDoubleAverage.getAverage();
        }
        double fps = totalMs > 0 ? 1000d / totalMs : -1;
        strings.add(String.format("FPS: %d", Math.round(fps)));

        // === Contours
        List<ResultContour> results = mainPipeline.getResults();
        if (results != null && results.size() >= 1) {
            strings.add(String.format("Contours (%d):", results.size()));
            for (ResultContour contour : results) {
                strings.add("- " + contour);
            }
        }

        int y = 20;
        for (String s : strings) {
            Imgproc.putText(mat, s, new Point(0, y), Core.FONT_HERSHEY_SIMPLEX, 0.4,
                    new Scalar(255, 255, 255), 1);
            y += 15;
        }
        return mat;
    }
}
