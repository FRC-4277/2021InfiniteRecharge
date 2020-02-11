package frc4277.vision.pipelines;

import frc4277.vision.pipelines.setting.Setting;
import org.opencv.core.Mat;

import java.util.List;

public class ErodeDilatePipeline extends Pipeline {
    public ErodeDilatePipeline() {
        super("ErodeDilate");
    }

    @Override
    public void process(Mat mat, Context context) {

    }

    @Override
    public List<Setting> getSettings() {
        return null;
    }
}
