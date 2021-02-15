package frc.robot.commands.autonomous.galactic;

public enum PathColor {
    RED(true), BLUE(false);
    private boolean powerCellClose;

    PathColor(boolean powerCellClose) {
        this.powerCellClose = powerCellClose;
    }

    public boolean isPowerCellClose() {
        return powerCellClose;
    }
}
