package frc4277.galacticvision.pipelines;

import edu.wpi.first.vision.VisionPipeline;
import frc4277.galacticvision.GalacticVision;
import frc4277.galacticvision.controllers.MainApplication;
import frc4277.galacticvision.util.RollingDoubleAverage;
import javafx.scene.image.Image;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.ByteArrayInputStream;
import java.util.*;
import java.util.function.Supplier;

public class MainPipeline implements VisionPipeline {
    private static final int ROLLING_AVERAGE_TIME_SIZE = 10;

    private final Supplier<Boolean> enabledSupplier;
    // LinkedHashMap to maintain order
    private final Map<PipelineType, Pipeline> pipelineMap = new LinkedHashMap<>();
    private final Map<PipelineType, RollingDoubleAverage> averageMap = new HashMap<>();
    private final Map<PipelineType, Mat> imageMap = new HashMap<>();
    private Mat normalImage;
    private List<ResultContour> results = new ArrayList<>();

    public MainPipeline(Supplier<Boolean> enabledSupplier) {
        this.enabledSupplier = enabledSupplier;
        for (PipelineType pipelineType : PipelineType.values()) {
            try {
                pipelineMap.put(pipelineType, pipelineType.getPipelineClass().getDeclaredConstructor().newInstance());
                averageMap.put(pipelineType, new RollingDoubleAverage(ROLLING_AVERAGE_TIME_SIZE));
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void process(Mat image) {
        MainApplication mainApplication = GalacticVision.getInstance().getMainApplication();
        if (mainApplication == null) {
            return;
        }
        if (!enabledSupplier.get()) {
            return;
        }

        normalImage = image.clone();

        // Run through pipelines
        for (Map.Entry<PipelineType, Pipeline> entry : pipelineMap.entrySet()) {
            PipelineType type = entry.getKey();
            Pipeline pipeline = entry.getValue();

            try {
                long startMs = System.currentTimeMillis();
                image = pipeline.processImage(mainApplication, image, this);
                imageMap.put(type, image.clone());
                long elapsedMs = System.currentTimeMillis() - startMs;
                averageMap.get(type).update(elapsedMs);
            } catch (Exception e) {
                System.out.println("Error in pipeline " + type.getName());
                e.printStackTrace();
            }
        }

        // Display image in ImageView
        String selected = mainApplication.getViewChoice();
        if (selected == null || selected.isEmpty()) {
            return;
        }

        Mat desiredMat = null;
        if (selected.equals("Normal")) {
            desiredMat = normalImage;
        }
        for (PipelineType type : PipelineType.values()) {
            if (selected.equals(type.getName())) {
                desiredMat = imageMap.get(type);
                break;
            }
        }

        if (desiredMat != null) {
            mainApplication.video.setImage(convertMatToImage(desiredMat));
        }
    }

    private Image convertMatToImage(Mat mat) {
        MatOfByte matOfByte = new MatOfByte();
        Imgcodecs.imencode(".bmp", mat, matOfByte);
        return new Image(new ByteArrayInputStream(matOfByte.toArray()));
    }

    public Map<PipelineType, RollingDoubleAverage> getAverageMap() {
        return averageMap;
    }

    public Mat getNormalImage() {
        return normalImage;
    }

    public List<ResultContour> getResults() {
        return results;
    }

    public void setResults(List<ResultContour> results) {
        this.results = results;
    }
}
