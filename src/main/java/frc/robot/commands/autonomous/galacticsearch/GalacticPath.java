package frc.robot.commands.autonomous.galacticsearch;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

import java.util.List;

public class GalacticPath {
    private PathColor color;
    private PathLetter letter;
    private Translation2d firstPowerCell;
    private Translation2d secondPowerCell;
    private Translation2d thirdPowerCell;

    public GalacticPath(PathColor color, PathLetter letter, Translation2d firstPowerCell,
                        Translation2d secondPowerCell, Translation2d thirdPowerCell) {
        this.color = color;
        this.letter = letter;
        this.firstPowerCell = firstPowerCell;
        this.secondPowerCell = secondPowerCell;
        this.thirdPowerCell = thirdPowerCell;
    }

    public static GalacticPath fromFeet(PathColor color, PathLetter letter, Translation2d firstPowerCell,
                                        Translation2d secondPowerCell, Translation2d thirdPowerCell) {
        Translation2d firstPowerCellM = new Translation2d(Units.feetToMeters(firstPowerCell.getX()),
                Units.feetToMeters(firstPowerCell.getY()));
        Translation2d secondPowerCellM = new Translation2d(Units.feetToMeters(secondPowerCell.getX()),
                Units.feetToMeters(secondPowerCell.getY()));
        Translation2d thirdPowerCellM = new Translation2d(Units.feetToMeters(thirdPowerCell.getX()),
                Units.feetToMeters(thirdPowerCell.getY()));

        return new GalacticPath(color, letter, firstPowerCellM, secondPowerCellM, thirdPowerCellM);
    }

    public PathColor getColor() {
        return color;
    }

    public PathLetter getLetter() {
        return letter;
    }

    public Translation2d getFirstPowerCell() {
        return firstPowerCell;
    }

    public Translation2d getSecondPowerCell() {
        return secondPowerCell;
    }

    public Translation2d getThirdPowerCell() {
        return thirdPowerCell;
    }

    public List<Translation2d> getThreePowerCells() {
        return List.of(firstPowerCell, secondPowerCell, thirdPowerCell);
    }

    public boolean isPowerCellClose() {
        return color.isPowerCellClose();
    }

    @Override
    public String toString() {
        return getLetter().name() + " - " + getColor().name();
    }
}
