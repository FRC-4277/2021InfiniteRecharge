package frc4277.vision.pipelines;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.vision.VisionPipeline;
import frc4277.vision.Main;
import org.opencv.core.Mat;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class MainPipeline implements VisionPipeline {
    private static final long UPDATE_STATISTICS_PERIOD_MS = 5000;

    private Main main;
    private NetworkTable statisticsTable;
    private long lastStatisticsUpdate = -1;
    private Map<Pipelines,RollingDoubleAverage> averagePipelinesMs = new HashMap<>();
    private RollingDoubleAverage averageFrameMs = new RollingDoubleAverage(30);

    public MainPipeline(Main main, NetworkTable statisticsTable) {
        this.main = main;
        this.statisticsTable = statisticsTable;

        for (Pipelines pipeline : Pipelines.values()) {
            averagePipelinesMs.put(pipeline, new RollingDoubleAverage(20));
        }
    }

    @Override
    public void process(Mat mat) {
        // Remember start time
        long startTime = System.currentTimeMillis();

        // Process through pipelines
        for (Pipelines pipelineEnum : Pipelines.values()) {
            // Remember pipeline start time
            long pipelineStartTime = System.currentTimeMillis();

            // Run pipeline
            try {
                Pipeline pipeline = pipelineEnum.getInstance();
                pipeline.process(mat);

                if (main.isPsEyeOutput() && Objects.equals(main.getPipelineOutput(), pipelineEnum)) {
                    // Must output this frame
                    main.addPipelineOutputFrame(mat);
                }
            } catch (Exception e) {
                System.out.println("Failed to run pipeline " + pipelineEnum.getInstance().getName());
                System.out.println("Continuing anyways..");
                e.printStackTrace();
            }

            // End of pipeline, do statistic
            averagePipelinesMs.get(pipelineEnum).update((System.currentTimeMillis() - pipelineStartTime));
        }

        // End, do statistics
        averageFrameMs.update((System.currentTimeMillis() - startTime));
    }

    public void printStatistics() {
        if ((System.currentTimeMillis() - lastStatisticsUpdate) <= UPDATE_STATISTICS_PERIOD_MS) {
            return;
        }
        lastStatisticsUpdate = System.currentTimeMillis();
        for (Map.Entry<Pipelines,RollingDoubleAverage> entry : averagePipelinesMs.entrySet()) {
            String name = entry.getKey().toString();
            statisticsTable.getEntry(name + "_time").setDouble(entry.getValue().getAverage());
        }
        double frameMs = averageFrameMs.getAverage();
        statisticsTable.getEntry("frame_time").setDouble(frameMs);
        statisticsTable.getEntry("fps").setDouble((1000d / frameMs));
    }
}
