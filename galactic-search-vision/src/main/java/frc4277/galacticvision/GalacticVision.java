package frc4277.galacticvision;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import frc4277.galacticvision.controllers.MainApplication;
import frc4277.galacticvision.pipelines.MainPipeline;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.image.Image;
import javafx.stage.Stage;

import java.io.IOException;
import java.net.InetAddress;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Objects;
import java.util.regex.Matcher;
import java.util.regex.Pattern;


public class GalacticVision extends Application {
    private static GalacticVision instance;
    private volatile NetworkTableInstance ntInstance;
    private Stage stage;
    private boolean enabled = false;
    private MainApplication mainApplication;
    private VisionThread visionThread;

    public static GalacticVision getInstance() {
        return instance;
    }

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    @SuppressWarnings("ConstantConditions")
    public void start(Stage stage) throws Exception {
        instance = this;
        Parent root = FXMLLoader.load(getClass().getClassLoader().getResource("main_application.fxml"));
        Scene scene = new Scene(root, 1000, 800);
        stage.setTitle("FRC 4277 - Galactic Vision (disabled)");
        stage.setScene(scene);
        stage.getIcons().addAll(
                new Image(getClass().getClassLoader().getResourceAsStream("icon-128.png")),
                new Image(getClass().getClassLoader().getResourceAsStream("icon-64.png")),
                new Image(getClass().getClassLoader().getResourceAsStream("icon-32.png"))
        );
        stage.show();
        this.stage = stage;

        // Connect to NetworkTables
        setupNetworkTables();
        // Watch for auto selector to be set to "Galactic Search"
        setupWatchForSearch();
    }

    private void setupNetworkTables() {
        ntInstance = NetworkTableInstance.getDefault();
        ntInstance.startClientTeam(4277);
        ntInstance.startDSClient();
    }

    private void setupWatchForSearch() {
        NetworkTableEntry selectorEntry = ntInstance.getTable("Shuffleboard/Autonomous/Autonomous Command")
                .getEntry("selected");
        selectorEntry.addListener(notification -> {
                    selectorUpdate(notification.value.getString());
                },
                EntryListenerFlags.kImmediate | EntryListenerFlags.kUpdate);
        new Thread(() -> {
            // Wait for connection
            try {
                while (!ntInstance.isConnected()) {
                    //noinspection BusyWait
                    Thread.sleep(100);
                }
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            selectorUpdate(selectorEntry.getString(""));
        }).start();
    }

    private void selectorUpdate(String value) {
        enabled = Objects.equals(value, "Galactic Search");
        Platform.runLater(() ->
                stage.setTitle("FRC 4277 - Galactic Vision (" + (enabled ? "enabled" : "disabled") + ")")
        );

        // Start vision thread if needed
        if (enabled && visionThread == null) {
            setupVisionThread();
        }
    }

    private void setupVisionThread() {
        HttpCamera httpCamera = findHttpCamera();
        if (httpCamera == null) {
            System.out.println("Could not start vision thread as no good http cameras found");
        }

        visionThread = new VisionThread(httpCamera, new MainPipeline(this::isEnabled), pipeline -> { });
        visionThread.start();
        System.out.println("Vision thread started!!!");
    }

    private HttpCamera findHttpCamera() {
        System.out.println("Finding HTTP camera for Switched");
        var urlsArray = ntInstance.getTable("CameraPublisher")
                .getSubTable("Switched").getEntry("streams").getStringArray(new String[]{});
        var urlsSet = new HashSet<>(Arrays.asList(urlsArray));
        Pattern extractAddress = Pattern.compile("^mjpg:http(s)?:\\/\\/(?<address>.*):.*");
        for (String url : urlsSet) {
            Matcher matcher = extractAddress.matcher(url.trim());
            if (!matcher.find()) {
                System.out.println("URL doesn't match pattern: [" + url + "]");
                continue;
            }
            String address = matcher.group("address");
            if (isAddressReachable(address)) {
                System.out.println("Found reachable address at " + address);
                Pattern extractUrl = Pattern.compile("^mjpg:(?<url>.*)");
                Matcher matcher2 = extractUrl.matcher(url);
                if (!matcher2.find()) {
                    System.out.println("Extract URL fail for url" + url);
                    continue;
                }
                String cameraUrl = matcher2.group("url");
                System.out.println("Camera url is " + cameraUrl);
                return new HttpCamera("switched", cameraUrl);
            } else {
                System.out.println("Address " + address + " is not reachable");
            }

        }
        return null;
    }

    private boolean isAddressReachable(String address) {
        try {
            return InetAddress.getByName(address).isReachable(5000);
        } catch (IOException e) {
            System.out.println("Error checking reachability to " + address);
            e.printStackTrace();
            return false;
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public MainApplication getMainApplication() {
        return mainApplication;
    }

    public void setMainApplication(MainApplication mainApplication) {
        this.mainApplication = mainApplication;
    }

    public NetworkTableInstance getNTInstance() {
        return ntInstance;
    }
}
