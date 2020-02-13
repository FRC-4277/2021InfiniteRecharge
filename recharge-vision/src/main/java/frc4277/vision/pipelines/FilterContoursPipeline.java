package frc4277.vision.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;

import java.util.List;

public class FilterContoursPipeline extends Pipeline {
    private ContourPipeline contourPipeline;

    public FilterContoursPipeline() {
        super("FilterContours");
    }

    public void setContourPipeline(ContourPipeline contourPipeline) {
        this.contourPipeline = contourPipeline;
    }

    @Override
    public void process(Mat mat, Context context) {
        List<MatOfPoint> contours = contourPipeline.foundContours;
        for (int contourIndex = 0; contourIndex < contours.size(); contourIndex++) {
            MatOfPoint contour = contours.get(contourIndex);

        }
    }
}
