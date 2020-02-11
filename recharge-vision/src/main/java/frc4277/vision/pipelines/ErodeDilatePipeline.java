package frc4277.vision.pipelines;

import frc4277.vision.pipelines.setting.Setting;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class ErodeDilatePipeline extends Pipeline {
    private Setting<Integer> closeWidth = new Setting<>("closeWidth", Integer.class, 3);
    private Setting<Integer> closeHeight = new Setting<>("closeHeight", Integer.class, 3);

    public ErodeDilatePipeline() {
        super("ErodeDilate");
    }

    @Override
    public void process(Mat mat, Context context) {
        double closeWidth = this.closeWidth.get();
        double closeHeight = this.closeHeight.get();

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_CROSS, new Size(closeWidth,closeHeight)); //todo : cache
        Imgproc.morphologyEx(mat, mat, Imgproc.MORPH_CLOSE, kernel);
    }

    @Override
    public List<Setting> getSettings() {
        return List.of(closeWidth, closeHeight);
    }
}
