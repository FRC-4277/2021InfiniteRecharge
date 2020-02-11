package frc4277.vision.pipelines;

import frc4277.vision.pipelines.setting.Setting;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class BlurPipeline extends Pipeline {
    private Setting<Double> blurWidthPercent = new Setting<>("size", Double.class, 0.0);
    private Setting<Double> blurHeightPercent = new Setting<>("size", Double.class, 0.0);


    BlurPipeline() {
        super("Blur");
    }

    @Override
    public void process(Mat mat, Context context) {
        double blurWidthPercent = this.blurWidthPercent.get();
        double blurHeightPercent = this.blurHeightPercent.get();
        double blurWidth = context.width * blurWidthPercent;
        double blurHeight = context.height * blurHeightPercent;

        if (blurWidth >= 1 || blurHeight >= 1) {
            Imgproc.blur(mat, mat, new Size(blurWidth, blurHeight));
        }
    }

    @Override
    public List<Setting> getSettings() {
        return List.of(blurWidthPercent, blurHeightPercent);
    }
}
