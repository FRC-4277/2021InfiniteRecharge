package frc.robot.commands.autonomous.galactic;

public enum PathLetter {
    A(true), B(false);
    private boolean threeBallsVisible;

    PathLetter(boolean threeBallsVisible) {
        this.threeBallsVisible = threeBallsVisible;
    }

    public boolean isThreeBallsVisible() {
        return threeBallsVisible;
    }
}
