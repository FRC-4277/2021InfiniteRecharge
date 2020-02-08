package frc4277.shuffleboard.plugin.hopper;

import java.io.IOException;

import edu.wpi.first.shuffleboard.api.widget.Description;
import edu.wpi.first.shuffleboard.api.widget.ParametrizedController;
import edu.wpi.first.shuffleboard.api.widget.SimpleAnnotatedWidget;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.Image;
import javafx.scene.layout.Pane;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;

@Description(name = "VerticalHopper", dataTypes = VerticalHopper.class)
@ParametrizedController("VerticalHopper.fxml")
public class VerticalHopperWidget extends SimpleAnnotatedWidget<VerticalHopper> {
    // Stop animation 40ms after speed is no longer not 0
    private static final long MAXIMUM_ANIMATION_REQUEST_AGE = 40;

    @FXML
    StackPane root;
    @FXML
    ResizableCanvas canvas;

    private Image background = loadImage("background.png");
    private Image[] cells = new Image[]{
            loadImage("ball_0.png"),
            loadImage("ball_1.png"),
            loadImage("ball_2.png"),
            loadImage("ball_3.png"),
            loadImage("ball_4.png")};
    private Image[] topArrow = new Image[]{loadImage("top_arrow_1.png"),loadImage("top_arrow_2.png")};
    private Image[] bottomArrow = new Image[]{loadImage("bottom_arrow_1.png"),loadImage("bottom_arrow_2.png")};
    private Image gate = loadImage("gate.png");
    private boolean animationLoop = false;
    private int animationPeriodMs;
    private long animationRequestTime = -1;

    @FXML
    private void initialize() {
        canvas.setStackPane(root);
        canvas.setDrawConsumer(this::draw);

        dataProperty().subscribe((observable, oldValue, newValue) -> draw());

        new Thread(() -> {
            Runnable update = this::draw;
            while (true) {
                long time = System.currentTimeMillis();
                if ((time - animationRequestTime) <= MAXIMUM_ANIMATION_REQUEST_AGE) {
                    try {
                        Thread.sleep(animationPeriodMs);
                    } catch (InterruptedException ignored) {  }
                    Platform.runLater(update);
                }
            }
        }, "Vertical Hopper Animation").start();
    }

    public void draw() {
        draw(canvas.getGraphicsContext2D());
    }

    public void draw(GraphicsContext graphicsContext) {
        int width = (int) Math.round(canvas.getWidth());
        int height = (int) Math.round(canvas.getHeight());

        VerticalHopper hopper = dataOrDefault.get();

        graphicsContext.setFill(Color.WHITE);
        graphicsContext.fillRect(0, 0, width, height);

        // Background
        graphicsContext.drawImage(background, 0, 0);

        // Cells
        boolean[] cellsPresent = hopper.getCellsPresent();
        for (int i = 0; i < 5; i++) {
            if (cellsPresent[i]) {
                graphicsContext.drawImage(cells[i], 0, 0);
            }
        }
        // Gate
        if (hopper.isGateClosed()) {
            graphicsContext.drawImage(gate, 0,0);
        }
        // Speed
        double speed = hopper.getSpeedRunning();
        if (speed > 0.05 || speed < -0.05) {
            animationPeriodMs = (int) (600 - (500 * Math.abs(speed)));
            Image[] animationArrows = speed > 0 ? topArrow : bottomArrow;
            Image arrow = animationArrows[getAnimationLoop() ? 0 : 1];
            graphicsContext.drawImage(arrow, 0, 0);
            // Set this so the Thread in initialize() will Thread.sleep for periodMs and draw again
            animationRequestTime = System.currentTimeMillis();
        }
    }

    public Image loadImage(String name) {
        try {
            return new Image(getClass().getResource(name).openStream());
        } catch (IOException e) {
            e.printStackTrace();
        }
        return null;
    }

    public boolean getAnimationLoop() {
        return animationLoop = !animationLoop;
    }

    @Override
    public Pane getView() {
        return root;
    }
}
