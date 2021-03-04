package frc.robot.commands.autonomous.galacticvideo;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GalacticSearchVideo;
import frc.robot.subsystems.vision.VisionSystem;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;

import java.util.List;
import java.util.Optional;

public class DetectPathCommand extends CommandBase {
    private final GalacticAutoVideoCommand galacticAutoVideoCommand;
    private final VisionSystem visionSystem;

    private static final int STABLE_LOOPS_REQUIRED = 7; // 20 ms * 7 = 140ms wait
    private int stableLoops;

    public DetectPathCommand(GalacticAutoVideoCommand galacticAutoVideoCommand, VisionSystem visionSystem) {
        this.galacticAutoVideoCommand = galacticAutoVideoCommand;
        this.visionSystem = visionSystem;
        addRequirements(visionSystem);
    }

    @Override
    public void initialize() {
        visionSystem.setUsingPixy(true);
        stableLoops = 0;
        galacticAutoVideoCommand.setPathDetected(null);
    }

    @Override
    public void execute() {
        if (RobotBase.isSimulation()) {
            GalacticPath path = visionSystem.getSimPathSelected();
            if (path == null) {
                galacticAutoVideoCommand.setMessage("[Detect Path] Please select simulation path");
            }
            galacticAutoVideoCommand.setPathDetected(path);
            return;
        }

        List<Pixy2CCC.Block> blocks = visionSystem.getBlocksList();
        // See if we see at least two balls
        if (blocks.size() >= 2) {
            stableLoops++;
            galacticAutoVideoCommand.setMessage("[Detect Path] Stable loops at "
                    + stableLoops + " out of " + STABLE_LOOPS_REQUIRED);
        } else {
            stableLoops = 0;
            galacticAutoVideoCommand.setMessage("[Detect Path] Only found " + blocks.size() + " blocks.. Waiting for 2+");
        }

        if (stableLoops >= STABLE_LOOPS_REQUIRED) {
            // We have 5 times in a row found at least 2 blocks, let's use the data now
            int blockCount = blocks.size();
            boolean threeBallsVisible = blockCount >= 3;

            Optional<Pixy2CCC.Block> largestBlockOptional = visionSystem.getLargestBlock();
            if (largestBlockOptional.isEmpty()) {
                galacticAutoVideoCommand.setMessage("[Detect Path] Largest block optional is empty! Should be impossible here.");
                return;
            }
            int largestArea = visionSystem.calculateArea(largestBlockOptional.get());
            boolean powerCellClose = largestArea > GalacticSearchVideo.PIXY_AREA_THRESHOLD_FOR_CLOSE_POWER_CELL;

            GalacticPath detectedPath = GalacticPaths.findPath(powerCellClose, threeBallsVisible);
            if (detectedPath != null) {
                galacticAutoVideoCommand.setMessage("[Detect Path] Detected path is " + detectedPath);
                galacticAutoVideoCommand.setPathDetected(detectedPath); // Found the path!!!!
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        visionSystem.setUsingPixy(false);
    }

    @Override
    public boolean isFinished() {
        return galacticAutoVideoCommand.getPathDetected() != null;
    }
}
