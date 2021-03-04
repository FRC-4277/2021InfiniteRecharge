package frc.robot.commands.autonomous.galacticsearch;

public enum PathColor {
    RED(true), BLUE(false);
    private final boolean powerCellClose;

    PathColor(boolean powerCellClose) {
        this.powerCellClose = powerCellClose;
    }

    public boolean isPowerCellClose() {
        return powerCellClose;
    }
}
