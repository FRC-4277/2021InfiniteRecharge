package frc4277.shuffleboard.plugin.hopper;

import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.layout.StackPane;

import java.util.function.Consumer;

public class ResizableCanvas extends Canvas {
    private Consumer<GraphicsContext> graphicsContextConsumer= null;

    public ResizableCanvas() {
        widthProperty().addListener(evt -> drawIfPossible());
        heightProperty().addListener(evt -> drawIfPossible());
    }

    public void setStackPane(StackPane stackPane) {
        this.widthProperty().bind(stackPane.widthProperty());
        this.heightProperty().bind(stackPane.heightProperty());
    }

    public void setDrawConsumer(Consumer<GraphicsContext> graphicsContextConsumer) {
        this.graphicsContextConsumer = graphicsContextConsumer;
    }

    public void drawIfPossible() {
        if (graphicsContextConsumer != null && getWidth() >= 1 && getHeight() >= 1) {
            graphicsContextConsumer.accept(getGraphicsContext2D());
        }
    }

    @Override
    public boolean isResizable() {
        return true;
    }

    @Override
    public double prefWidth(double height) {
        return getWidth();
    }

    @Override
    public double prefHeight(double width) {
        return getHeight();
    }
}
