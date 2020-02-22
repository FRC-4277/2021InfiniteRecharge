package frc4277.shuffleboard.plugin.timer;

import edu.wpi.first.shuffleboard.api.prefs.Group;
import edu.wpi.first.shuffleboard.api.prefs.Setting;
import edu.wpi.first.shuffleboard.api.widget.Description;
import edu.wpi.first.shuffleboard.api.widget.ParametrizedController;
import edu.wpi.first.shuffleboard.api.widget.SimpleAnnotatedWidget;
import javafx.beans.property.*;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.geometry.Insets;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.Text;
import javafx.scene.text.TextFlow;

import java.util.List;
import java.util.function.Consumer;

@Description(name = "GameTimer", dataTypes = GameTimer.class)
@ParametrizedController("GameTimer.fxml")
public class GameTimerWidget extends SimpleAnnotatedWidget<GameTimer> {
    @FXML
    public Pane root;

    @FXML
    public TextFlow textFlow;

    private Text text;

    private StringProperty font = new SimpleStringProperty(this, "Regular");
    private DoubleProperty fontSize = new SimpleDoubleProperty(this, "fontSize", 70.0);
    private Property<Color> normalColor = new SimpleObjectProperty<>(this, "normalColor", Color.WHITE);
    private Property<Color> warningColor = new SimpleObjectProperty<>(this, "warningColor", Color.RED);
    private DoubleProperty verticalPadding = new SimpleDoubleProperty(this, "verticalPadding", 0.0);
    private BooleanProperty showMilliseconds = new SimpleBooleanProperty(this, "showMilliseconds", false);

    @Override
    public String getTitle() {
        return null;
    }

    @FXML
    public void initialize() {
        text = new Text("0:00:00");

        Runnable updateFont = () -> text.setFont(Font.font(font.get(), fontSize.get()));
        updateFont.run();
        font.addListener((observable, oldValue, newValue) -> updateFont.run());
        fontSize.addListener((observable, oldValue, newValue) -> updateFont.run());

        text.setFill(Color.GRAY);

        Consumer<Double> paddingConsumer = padding -> textFlow.setPadding(new Insets(padding, 0, padding, 0));
        paddingConsumer.accept((verticalPadding.get()));
        verticalPadding.addListener((observable, oldValue, newValue) -> {
            if (newValue == null) {
                return;
            }
            paddingConsumer.accept(newValue.doubleValue());
        });

        textFlow.getChildren().add(text);

        // Auto update
        dataProperty().subscribe((observable, oldValue, newValue) -> {
            if (newValue == null) {
                return;
            }
            // In seconds (decimal)
            double matchTime = newValue.getMatchTime();

            int minutes = Double.valueOf(Math.floor(matchTime / 60)).intValue();
            matchTime -= minutes * 60;

            int seconds = (int) Math.floor(matchTime);
            matchTime -= seconds;

            // 0.00..0.99 times 100 = 0..99
            int milliseconds = (int) Math.floor(matchTime * 100);

            if (minutes == 0 && seconds <= 30) {
                // Flash animation
                if (seconds % 2 == 0) {
                    text.setFill(warningColor.getValue());
                } else {
                    text.setFill(normalColor.getValue());
                }
            } else {
                // Normal game time
                text.setFill(normalColor.getValue());
            }

            if (showMilliseconds.get()) {
                text.setText(String.format("%d:%02d:%02d", minutes, seconds, milliseconds));
            } else {
                text.setText(String.format("%d:%02d", minutes, seconds));
            }
        });
    }

    @Override
    public List<Group> getSettings() {
        return List.of(
            Group.of("Appearance",
                Setting.of("Font", font, String.class),
                Setting.of("Font Size", fontSize, Double.class),
                Setting.of("Font Color", normalColor, Color.class),
                Setting.of("Warning Color", warningColor, Color.class),
                Setting.of("Vertical Padding", verticalPadding, Double.class),
                Setting.of("Show Milliseconds", showMilliseconds, Boolean.class)
            )
        );
    }

    @Override
    public Pane getView() {
        return root;
    }
}
