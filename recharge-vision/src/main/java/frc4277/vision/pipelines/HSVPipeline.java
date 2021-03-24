package frc4277.vision.pipelines;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc4277.vision.pipelines.setting.Setting;
import java.util.List;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class HSVPipeline extends Pipeline {
  private Setting<Integer> hMin =
      new Setting<>("h_min", Integer.class, 50, BuiltInWidgets.kTextView);
  private Setting<Integer> hMax =
      new Setting<>("h_max", Integer.class, 180, BuiltInWidgets.kTextView);
  private Setting<Integer> sMin =
      new Setting<>("s_min", Integer.class, 0, BuiltInWidgets.kTextView);
  private Setting<Integer> sMax =
      new Setting<>("s_max", Integer.class, 255, BuiltInWidgets.kTextView);
  private Setting<Integer> vMin =
      new Setting<>("v_min", Integer.class, 50, BuiltInWidgets.kTextView);
  private Setting<Integer> vMax =
      new Setting<>("v_max", Integer.class, 255, BuiltInWidgets.kTextView);

  HSVPipeline() {
    super("HSV");
  }

  @Override
  public void process(Mat mat, Context context) {
    // 3 = 3 channels (H, S, V)
    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2HSV, 3);
    // HSV Threshold
    Core.inRange(
        mat,
        new Scalar(hMin.get(), sMin.get(), vMin.get()),
        new Scalar(hMax.get(), sMax.get(), vMax.get()),
        mat);
  }

  @Override
  public List<Setting<?>> getSettings() {
    return List.of(hMin, hMax, sMin, sMax, vMin, vMax);
  }
}
