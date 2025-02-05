package frc.team5115.subsystems.bling;

import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

public class BlingIOSim implements BlingIO {
    private final int[][] leds; // RGBW
    private boolean running = false;

    public BlingIOSim() {
        leds = new int[Bling.LED_COUNT][4];
    }

    @Override
    public void setRGBW(int index, int red, int green, int blue, int white) {
        leds[index][0] = red;
        leds[index][1] = green;
        leds[index][2] = blue;
        leds[index][3] = white;
    }

    @Override
    public void start() {
        running = true;
    }

    @Override
    public void stop() {
        running = false;
    }

    @Override
    public void updateInputs(BlingIOInputs inputs) {
        inputs.ledStrip = leds; // set the references
        inputs.running = running;
        Logger.recordOutput("Bling/Sim/RedPoints", generatePoints(0));
        Logger.recordOutput("Bling/Sim/GreenPoints", generatePoints(1));
        Logger.recordOutput("Bling/Sim/BluePoints", generatePoints(2));
        Logger.recordOutput("Bling/Sim/WhitePoints", generatePoints(3));
        Logger.recordOutput("Bling/Origin", new Translation2d(0, 0));
    }

    private Translation2d[] generatePoints(int colorIndex) {
        final var points = new Translation2d[Bling.LED_COUNT];
        for (int i = 0; i < Bling.LED_COUNT; i++) {
            points[i] = new Translation2d(i * 2, leds[i][colorIndex]);
        }
        return points;
    }
}
