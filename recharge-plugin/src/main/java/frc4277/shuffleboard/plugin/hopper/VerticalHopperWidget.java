package frc4277.shuffleboard.plugin.hopper;

import java.io.IOException;

import edu.wpi.first.shuffleboard.api.widget.Description;
import edu.wpi.first.shuffleboard.api.widget.ParametrizedController;
import edu.wpi.first.shuffleboard.api.widget.SimpleAnnotatedWidget;
import javafx.concurrent.Task;
import javafx.fxml.FXML;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.Image;
import javafx.scene.layout.Pane;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;

@Description(name = "VerticalHopper", dataTypes = VerticalHopper.class)
@ParametrizedController("VerticalHopper.fxml")
public class VerticalHopperWidget extends SimpleAnnotatedWidget<VerticalHopper> {
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

    @FXML
    private void initialize() {
        canvas.setStackPane(root);
        canvas.setDrawConsumer(this::draw);

        dataProperty().subscribe((observable, oldValue, newValue) -> draw());
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
        if (speed > 0) {
            // Going up
            int periodMs = (int) (600 - (500 * speed));
            Image arrow = topArrow[getAnimationLoop() ? 0 : 1];
            graphicsContext.drawImage(arrow, 0,0);
            delay(this::draw, periodMs);
        } else if (speed < 0) {
            // Going down
            int periodMs = (int) (600 - (500 * Math.abs(speed)));
            Image arrow = bottomArrow[getAnimationLoop() ? 0 : 1];
            graphicsContext.drawImage(arrow, 0,0);
            delay(this::draw, periodMs);
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

    public void delay(Runnable runnable, int ms) {
        Task<Void> sleeper = new Task<>() {
            @Override
            protected Void call() {
                try {
                    Thread.sleep(ms);
                } catch (InterruptedException ignored) {
                }
                return null;
            }
        };
        sleeper.setOnSucceeded(event -> runnable.run());
        sleeper.run();
    }

    /*public void drawLine(Graphics2D graphics2D, Line line) {
        graphics2D.drawLine(line.x1, line.y1, line.x2, line.y2);
    }

    public class Line {
        public int x1, y1, x2, y2;

        public Line(int x1, int y1, int x2, int y2) {
            this.x1 = x1;
            this.y1 = y1;
            this.x2 = x2;
            this.y2 = y2;
        }
    }

    public class Render {
        public int width, height;
        public PulleyRender pulleyRender;
        public List<CellRender> cellRenders;
        public GateRender gateRender;
    }

    public class PulleyRender {
        public Line topLine, bottomLine;
        public int thickness = 5;
    }

    public class CellRender {
        // Top Left
        public int x1, y1;
        public int width, height;

        public CellRender(int x1, int y1, int width, int height) {
            this.x1 = x1;
            this.y1 = y1;
            this.width = width;
            this.height = height;
        }
    }

    public class GateRender {
        public Line line;
        public int thickness = 4;
    }*/

    @Override
    public Pane getView() {
        return root;
    }
}
