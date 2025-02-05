package frc.team5115.subsystems.bling;

import org.littletonrobotics.junction.AutoLog;

public interface BlingIO {
    @AutoLog
    public static class BlingIOInputs {
        public int[][] ledStrip = null; // RGBW
        public boolean running = false;
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(BlingIOInputs inputs) {}

    /**
     * Set the color of an LED
     *
     * @param index the index of the LED
     * @param red red component (0-255)
     * @param green green component (0-255)
     * @param blue blue component (0-255)
     * @param white white component (0-255)
     */
    public default void setRGBW(int index, int red, int green, int blue, int white) {}

    /** Starts the output */
    public default void start() {}

    /** Stop the output */
    public default void stop() {}
}
