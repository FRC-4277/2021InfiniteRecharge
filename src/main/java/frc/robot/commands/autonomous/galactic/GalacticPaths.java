package frc.robot.commands.autonomous.galactic;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

import java.util.List;

public class GalacticPaths {
    // Provided in feet, code will convert to meters

    public static final GalacticPath A_RED = GalacticPath.fromFeet(
            PathColor.RED,
            PathLetter.A,
            new Translation2d(7.5, 7.5),
            new Translation2d(12.5, 5.0),
            new Translation2d(15, 12.5)
    );

    public static final GalacticPath A_BLUE = GalacticPath.fromFeet(
            PathColor.BLUE,
            PathLetter.A,
            new Translation2d(15.0, 2.5),
            new Translation2d(17.5, 10.0),
            new Translation2d(22.5, 7.5)
    );

    public static final GalacticPath B_RED = GalacticPath.fromFeet(
            PathColor.RED,
            PathLetter.B,
            new Translation2d(7.5, 10.0),
            new Translation2d(12.5, 5.0),
            new Translation2d(17.5, 10.0)
    );

    public static final GalacticPath B_BLUE = GalacticPath.fromFeet(
            PathColor.BLUE,
            PathLetter.B,
            new Translation2d(15.0, 5.0),
            new Translation2d(20.0+.75, 10.0),
            new Translation2d(25.0+.75, 5.0)
    );

    public static List<GalacticPath> getAllPaths() {
        return List.of(A_RED, A_BLUE, B_RED, B_BLUE);
    }

    public static GalacticPath findPath(boolean powerCellClose, boolean threeBallsVisible) {
        for (GalacticPath path : getAllPaths()) {
            if (path.getColor().isPowerCellClose() == powerCellClose
                    && path.getLetter().isThreeBallsVisible() == threeBallsVisible) {
                return path;
            }
        }
        return null;
    }
}
