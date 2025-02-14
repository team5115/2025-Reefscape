package frc.team5115.subsystems.bling;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class BlingIOReal implements BlingIO {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final int[][] postBuffer; // GRB
    private final int[][] preBuffer; // RGBW
    private boolean running = false;

    public BlingIOReal() {
        led = new AddressableLED(Bling.LED_PORT);
        ledBuffer = new AddressableLEDBuffer(Bling.LED_COUNT * 4 / 3);
        preBuffer = new int[Bling.LED_COUNT][4];
        postBuffer = new int[ledBuffer.getLength()][3];
        led.setLength(ledBuffer.getLength());
    }

    @Override
    public void updateInputs(BlingIOInputs inputs) {
        inputs.running = running;
        inputs.ledStrip = preBuffer; // set the reference
        // Copy the postBuffer into the ledBuffer, then push ledBuffer to led
        for (int i = 0; i < postBuffer.length; i++) {
            ledBuffer.setRGB(i, postBuffer[i][1], postBuffer[i][0], postBuffer[i][2]);
        }
        led.setData(ledBuffer);
    }

    @Override
    public void setRGBW(int index, int red, int green, int blue, int white) {
        preBuffer[index][0] = red;
        preBuffer[index][1] = green;
        preBuffer[index][2] = blue;
        preBuffer[index][3] = white;

        // Transfer the preBuffer into the postBuffer; maps from RGBW to GRB
        final int mod = index % 3;
        index += index / 3;
        postBuffer[index + 0][0 + mod] = green;
        postBuffer[index + (mod >= 2 ? 1 : 0)][(1 + mod) % 3] = red;
        postBuffer[index + (mod >= 1 ? 1 : 0)][(2 + mod) % 3] = blue;
        postBuffer[index + 1][0 + mod] = white;
    }

    @Override
    public void start() {
        led.start();
        running = true;
    }

    @Override
    public void stop() {
        led.stop();
        running = false;
    }
}
