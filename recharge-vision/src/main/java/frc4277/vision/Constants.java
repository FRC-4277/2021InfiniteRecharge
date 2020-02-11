package frc4277.vision;

import edu.wpi.first.networktables.EntryListenerFlags;

public class Constants {
    // 5800-5810 is for team use
    // Remember 4 mb/s limit
    public static final int PSEYE_OUTPUT_STREAM_PORT = 5800;
    public static final int PSEYE_DEFAULT_FPS = 100;
    public static final int PSEYE_WIDTH = 320 * 2;
    public static final int PSEYE_HEIGHT = 240 * 2;
    public static final int PSEYE_OUTPUT_FPS = 30;

    public static final int NT_UPDATE_FLAGS = EntryListenerFlags.kImmediate | EntryListenerFlags.kNew | EntryListenerFlags.kUpdate;
}
