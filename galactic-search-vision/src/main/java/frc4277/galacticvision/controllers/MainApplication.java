package frc4277.galacticvision.controllers;


import frc4277.galacticvision.GalacticVision;
import frc4277.galacticvision.pipelines.PipelineType;
import javafx.fxml.FXML;
import javafx.scene.control.ChoiceBox;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;

public class MainApplication {
    @FXML
    public ImageView video;
    @FXML
    public TextField blurW;
    @FXML
    public TextField blurH;
    @FXML
    public TextField gaussianBlurW;
    @FXML
    public TextField gaussianBlurH;
    @FXML
    public TextField hMin;
    @FXML
    public TextField hMax;
    @FXML
    public TextField vMin;
    @FXML
    public TextField vMax;
    @FXML
    public TextField sMin;
    @FXML
    public TextField sMax;
    @FXML
    public TextField erosionW;
    @FXML
    public TextField erosionH;
    @FXML
    public TextField dilationW;
    @FXML
    public TextField dilationH;
    @FXML
    public TextField openW;
    @FXML
    public TextField openH;
    @FXML
    public TextField closeW;
    @FXML
    public TextField closeH;
    @FXML
    public TextField erosion2W;
    @FXML
    public TextField erosion2H;
    @FXML
    public TextField dilation2W;
    @FXML
    public TextField dilation2H;

    @FXML
    public ChoiceBox<String> viewChoiceBox;

    //private StringProperty topTextString = new SimpleStringProperty();

    @FXML
    public void initialize() {
        GalacticVision.getInstance().setMainApplication(this);

        viewChoiceBox.getItems().add("Normal");
        for (PipelineType type : PipelineType.values()) {
            viewChoiceBox.getItems().add(type.getName());
        }
        viewChoiceBox.setValue("Normal");

        System.out.println("Main application FXML initialized");
    }

    public ImageView getVideo() {
        return video;
    }

    public int getInteger(TextField textField) {
        String text = textField.getText();
        try {
            return Integer.parseInt(text);
        } catch (NumberFormatException e) {
            return 0;
        }
    }

    public double getDouble(TextField textField) {
        String text = textField.getText();
        try {
            return Double.parseDouble(text);
        } catch (NumberFormatException e) {
            return 0;
        }
    }

    public String getViewChoice() {
        return viewChoiceBox.getValue();
    }
}
