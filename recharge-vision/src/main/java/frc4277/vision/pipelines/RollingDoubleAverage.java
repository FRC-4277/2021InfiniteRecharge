package frc4277.vision.pipelines;

import java.util.LinkedList;

public class RollingDoubleAverage {
  private LinkedList<Double> queue;
  private int size;
  private int sum = 0;

  public RollingDoubleAverage(int size) {
    this.queue = new LinkedList<>();
    this.size = size;
  }

  @SuppressWarnings({"ConstantConditions"})
  public void update(double next) {
    queue.offer(next);
    sum += next;
    if (queue.size() > size) {
      sum -= queue.poll();
    }
  }

  public double getAverage() {
    return sum / (double) queue.size();
  }
}
