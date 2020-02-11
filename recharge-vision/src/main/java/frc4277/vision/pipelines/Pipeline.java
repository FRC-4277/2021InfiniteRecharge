package frc4277.vision.pipelines;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.vision.VisionPipeline;
import frc4277.vision.pipelines.setting.Setting;
import org.opencv.core.Mat;

import java.util.Collection;
import java.util.Collections;
import java.util.List;

public abstract class Pipeline implements VisionPipeline {
    private String name;
    private NetworkTable table;

    public Pipeline(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public NetworkTable getTable() {
        return table;
    }

    public void setTable(NetworkTable table) {
        this.table = table;
    }

    public abstract void process(Mat mat, Context context);

    @Override
    public void process(Mat mat) {
        process(mat, new Context(mat.width(), mat.height()));
    }

    public List<Setting> getSettings() {
        return Collections.emptyList();
    }

    public static class Context {
        public int width;
        public int height;

        public Context(int width, int height) {
            this.width = width;
            this.height = height;
        }
    }
}
