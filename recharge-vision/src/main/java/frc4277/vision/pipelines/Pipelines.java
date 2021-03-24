package frc4277.vision.pipelines;

public enum Pipelines {
  BLUR(BlurPipeline.class, new BlurPipeline()),
  HSV(HSVPipeline.class, new HSVPipeline()),
  ERODE_DILATE(ErodeDilatePipeline.class, new ErodeDilatePipeline()),
  CONTOUR(ContourPipeline.class, new ContourPipeline());

  private Class<? extends Pipeline> pipelineClass;
  private Pipeline instance;

  Pipelines(Class<? extends Pipeline> pipelineClass, Pipeline instance) {
    this.pipelineClass = pipelineClass;
    this.instance = instance;
  }

  public Class<? extends Pipeline> getPipelineClass() {
    return pipelineClass;
  }

  public Pipeline getInstance() {
    return instance;
  }

  @Override
  public String toString() {
    return instance.getName();
  }
}
