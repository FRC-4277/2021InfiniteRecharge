package frc4277.galacticvision.pipelines;

public enum PipelineType {
  BLUR("Blur", BlurPipeline.class),
  GAUSSIAN_BLUR("Gaussian Blur", GaussianBlurPipeline.class),
  HSV_THRESHOLD("HSV", HSVPipeline.class),
  EROSION("Erosion1", ErosionPipeline.class),
  DILATION("Dilation1", DilationPipeline.class),
  OPEN("Open", OpenPipeline.class),
  CLOSE("Close", ClosePipeline.class),
  EROSION2("Erosion2", Erosion2Pipeline.class),
  DILATION2("Dilation2", Dilation2Pipeline.class),
  CONTOURS("Contours", ContourPipeline.class),
  STATISTICS("Information", StatsPipeline.class),
  OVERLAY("Overlay", OverlayPipeline.class);
  private final String name;
  private final Class<? extends Pipeline> pipelineClass;

  PipelineType(String name, Class<? extends Pipeline> pipelineClass) {
    this.name = name;
    this.pipelineClass = pipelineClass;
  }

  public Class<? extends Pipeline> getPipelineClass() {
    return pipelineClass;
  }

  public String getName() {
    return name;
  }
}
