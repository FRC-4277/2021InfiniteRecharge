package frc.robot.subsystems.vision.limelight;

public class Pipeline {
  private final String name;
  private final int id;

  public Pipeline(String name, int id) {
    this.name = name;
    this.id = id;
  }

  public String getName() {
    return name;
  }

  public int getId() {
    return id;
  }
}
