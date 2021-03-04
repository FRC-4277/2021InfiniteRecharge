package frc.robot.commands.autonomous.galacticsearch;

public enum PathLetter {
    A(true), B(false);
    private final boolean threeBallsDifferentX;

    PathLetter(boolean threeBallsDifferentX) {
        this.threeBallsDifferentX = threeBallsDifferentX;
    }

    public boolean isThreeBallsDifferentX() {
        return threeBallsDifferentX;
    }
}
