package frc.team5115.subsystems.bling;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Bling extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final byte[][] buffer; // GRB

    public Bling() {
        led = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(192);

        buffer = new byte[ledBuffer.getLength()][3];

        led.setLength(ledBuffer.getLength());

        setRGBW(3, 100, 0, 0, 0);
        setRGBW(4, 0, 100, 0, 0);
        setRGBW(5, 0, 0, 100, 0);
        setRGBW(6, 0, 0, 0, 100);
        loadData();
        led.start();
    }

    public void setRGBW(int i, byte r, byte g, byte b, byte w) {
        int offset = i / 3;
        int mod = i % 3;
        i += offset;

        buffer[i + 0][0 + mod] = g;
        buffer[i + (mod >= 2 ? 1 : 0)][(1 + mod) % 3] = r;
        buffer[i + (mod >= 1 ? 1 : 0)][(2 + mod) % 3] = b;
        buffer[i + 1][0 + mod] = w;
    }

    public void setRGBW(int i, int r, int g, int b, int w) {
        setRGBW(i, (byte) r, (byte) g, (byte) b, (byte) w);
    }

    public void loadData() {
        for (int i = 0; i < buffer.length; i++) {
            ledBuffer.setRGB(i, buffer[i][1], buffer[i][0], buffer[i][2]);
        }
        led.setData(ledBuffer);
    }

    public Command knightRider(){
        return Commands.run(() -> {
            
        });
    }
}
