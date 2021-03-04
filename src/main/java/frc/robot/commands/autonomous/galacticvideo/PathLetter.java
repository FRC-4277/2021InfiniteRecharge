package frc.robot.commands.autonomous.galacticvideo;

public enum PathLetter {
    A(true), B(false);
    private final boolean threeBallsVisible;

    PathLetter(boolean threeBallsVisible) {
        this.threeBallsVisible = threeBallsVisible;
    }

    public boolean isThreeBallsVisible() {
        return threeBallsVisible;
    }
}
