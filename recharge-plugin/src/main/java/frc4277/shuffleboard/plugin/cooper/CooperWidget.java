package frc4277.shuffleboard.plugin.cooper;

import edu.wpi.first.shuffleboard.api.widget.Description;
import edu.wpi.first.shuffleboard.api.widget.ParametrizedController;
import edu.wpi.first.shuffleboard.api.widget.SimpleAnnotatedWidget;
import javafx.fxml.FXML;
import javafx.scene.layout.Pane;

@Description(name = "Cooper", dataTypes = Cooper.class)
@ParametrizedController("Cooper.fxml")
public class CooperWidget extends SimpleAnnotatedWidget<Double> {
    @FXML
    public Pane root;

    @Override
    public Pane getView() {
        return root;
    }
}
