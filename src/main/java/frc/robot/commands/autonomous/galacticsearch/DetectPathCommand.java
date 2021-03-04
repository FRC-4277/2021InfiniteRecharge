package frc.robot.commands.autonomous.galacticsearch;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GalacticSearch;
import frc.robot.subsystems.vision.GalacticVision;
import frc.robot.subsystems.vision.VisionSystem;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class DetectPathCommand extends CommandBase {
    private final GalacticAutoCommand galacticAutoCommand;
    private final VisionSystem visionSystem;

    public DetectPathCommand(GalacticAutoCommand galacticAutoCommand, VisionSystem visionSystem) {
        this.galacticAutoCommand = galacticAutoCommand;
        this.visionSystem = visionSystem;
        addRequirements(visionSystem);
    }

    @Override
    public void initialize() {
        galacticAutoCommand.setPathDetected(null);
    }

    @Override
    public void execute() {
        if (RobotBase.isSimulation()) {
            GalacticPath path = visionSystem.getSimPathSelected2();
            if (path == null) {
                galacticAutoCommand.setMessage("[Detect Path] Please select simulation path");
            }
            galacticAutoCommand.setPathDetected(path);
        }

        GalacticVision galacticVision = visionSystem.getGalacticVision();
        List<GalacticVision.PowerCellTarget> targets = galacticVision.getLargestPowerCells();
        if (targets.size() < 3) {
            galacticAutoCommand.setMessage("[Detect Path] Only found " + targets.size() + " power cells, waiting...");
            return;
        }
        GalacticVision.PowerCellTarget largestPowerCell = targets.get(0);

        // TRUE for red, FALSE for blue
        boolean powerCellClose = largestPowerCell.getArea() > GalacticSearch.VISION_AREA_THRESHOLD_FOR_CLOSE_POWER_CELL;
        SmartDashboard.putNumber("[GS] Area", largestPowerCell.getArea());
        // TRUE for A, FALSE for B
        boolean threeBallsDifferentX = areXCoordinatesDifferent(targets);
        galacticAutoCommand.setMessage("[Detect Path] Power cell close: " + powerCellClose + ", diff x: " + threeBallsDifferentX);

        GalacticPath detectedPath = GalacticPaths.findPath(powerCellClose, threeBallsDifferentX);
        if (detectedPath != null) {
            galacticAutoCommand.setMessage("[Detect Path] Detected path is " + detectedPath);
            SmartDashboard.putString("[GS] Path", detectedPath.toString());
            if (RobotBase.isReal()) {
                galacticAutoCommand.setPathDetected(detectedPath); // Found the path!!!!
            }
        }
    }

    private boolean areXCoordinatesDifferent(List<GalacticVision.PowerCellTarget> targets) {
        List<GalacticVision.PowerCellTarget> targetsXSorted = new ArrayList<>(targets);
        // Ascending order: left to right
        targetsXSorted.sort(Comparator.comparingDouble(GalacticVision.PowerCellTarget::getX));
        double distanceLeftToMiddle = Math.abs(targets.get(1).getX() - targets.get(0).getX());
        double distanceRightToMiddle = Math.abs(targets.get(2).getX() - targets.get(1).getX());
        galacticAutoCommand.setMessage("[Detect Path] Left distance to mid: " + distanceLeftToMiddle);
        SmartDashboard.putNumber("[GS] Left Distance", distanceLeftToMiddle);
        galacticAutoCommand.setMessage("[Detect Path] Right distance to mid: " + distanceRightToMiddle);
        SmartDashboard.putNumber("[GS] Right Distance", distanceRightToMiddle);
        return distanceLeftToMiddle >= GalacticSearch.VISION_DIFFERENT_X_THRESHOLD &&
                distanceRightToMiddle >= GalacticSearch.VISION_DIFFERENT_X_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return galacticAutoCommand.getPathDetected() != null;
    }
}
