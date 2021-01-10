package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.RunVerificationsCommand;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@SuppressWarnings("unused") // TODO : Implement more verifications
public class VerificationSystem extends SubsystemBase {
  private DriveTrain driveTrain;
  private Intake intake;
  private VerticalHopper verticalHopper;
  private Shooter shooter;
  private ColorWheel colorWheel;
  private CameraSystem cameraSystem;
  private VisionSystem visionSystem;
  private Winch winch;
  private HookElevator hookElevator;
  private List<VerifiableSystem> systems;
  private ShuffleboardTab tab;
  private Map<VerifiableSystem, ShuffleboardLayout> layoutMap = new HashMap<>();
  private Map<VerifiableSystem, List<Verification>> verificationsMap = new HashMap<>();
  private NetworkTableEntry errorsEntry;
  private String errors = "";

  public VerificationSystem(DriveTrain driveTrain, Intake intake, VerticalHopper verticalHopper, Shooter shooter,
                            ColorWheel colorWheel, CameraSystem cameraSystem, VisionSystem visionSystem, Winch winch,
                            HookElevator hookElevator, ShuffleboardTab tab) {
    this.driveTrain = driveTrain;
    this.intake = intake;
    this.verticalHopper = verticalHopper;
    this.shooter = shooter;
    this.colorWheel = colorWheel;
    this.cameraSystem = cameraSystem;
    this.visionSystem = visionSystem;
    this.winch = winch;
    this.hookElevator = hookElevator;
    this.tab = tab;
    this.systems = List.of(driveTrain, intake, verticalHopper, shooter, colorWheel, cameraSystem, visionSystem,
            winch, hookElevator);
    initializeTab();
  }

  private void initializeTab() {
    int rowIndex = 0;
    int columnIndex = 0;
    for (VerifiableSystem system : systems) {
      List<Verification> verifications = system.getVerifications(this);
      verificationsMap.put(system, verifications);

      // Generate layout and add to map
      String name = system.getClass().getSimpleName();
      ShuffleboardLayout layout = tab
              .getLayout(name, BuiltInLayouts.kGrid)
              .withProperties(Map.of(
            "Label position", "TOP",
            "Number of columns", (verifications == null ? 0 : verifications.size()),
            "Number of rows", 1
              ))
              .withPosition(columnIndex, rowIndex)
              .withSize(4, 1);
      layoutMap.put(system, layout);

      // Generate widgets
      if (verifications != null) {
        for (Verification verification : verifications) {
          NetworkTableEntry entry = layout.add(verification.getName(), false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(1, 1)
            .getEntry();
          verification.setWidgetEntry(entry);
        }
      }

      // Increment indexes
      if (columnIndex == 0) {
        columnIndex = 4;
      } else {
        rowIndex++;
        columnIndex = 0;
      }
    }
    if (columnIndex == 4) {
      rowIndex++;
    }

    tab.add("Run", new RunVerificationsCommand(this))
    .withPosition(0, rowIndex)
    .withSize(1, 1);

    errorsEntry = tab.add("Errors", "NOT RAN YET")
    .withWidget(BuiltInWidgets.kTextView)
    .withPosition(1, rowIndex)
    .withSize(7, 1)
    .getEntry();
  }

  public void error(String msg) {
    if (errors.equals("")) {
      errors = msg;
    } else {
      errors += "|" + msg;
    }
  }

  public void runVerifications() {
    errors = "";
    for (VerifiableSystem system : systems) {
      if (!verificationsMap.containsKey(system)) {
        continue;
      }
      List<Verification> verifications = verificationsMap.get(system);

      for (Verification verification : verifications) {
        NetworkTableEntry entry = verification.getWidgetEntry();
        if (entry == null) {
          continue;
        }
        entry.setBoolean(verification.check());
      }
    }
    errorsEntry.setString(errors);
  }
}
