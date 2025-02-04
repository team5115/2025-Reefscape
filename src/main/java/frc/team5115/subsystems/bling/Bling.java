package frc.team5115.subsystems.bling;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Bling extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final double[] prevPercent;
    private final byte[][] buffer; // GRB
    private final int ledCount;

    private final int period = 1;
    private final double tailLength = 20;
    private final double decay = 1d / tailLength;
    private final int minPower = 10;
    private final int maxPower = 255;

    private int timer_idle;
    private int counter_idle;
    private int direction_idle;

    public Bling() {
        ledCount = 144;
        led = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(ledCount * 4 / 3);
        buffer = new byte[ledBuffer.getLength()][3];
        prevPercent = new double[ledCount];
        led.setLength(ledBuffer.getLength());

        timer_idle = 0;
        counter_idle = 0;
        direction_idle = 1;
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

    @Override
    public void periodic() {
        Logger.recordOutput("Bling/TimerIdle", timer_idle);
        Logger.recordOutput("Bling/CounterIdle", counter_idle);
        Logger.recordOutput("Bling/DirectionIdle", direction_idle);

        timer_idle++;
        if (timer_idle >= period) {
            timer_idle = 0;
        } else {
            return;
        }

        counter_idle += direction_idle * 2;
        if (counter_idle >= ledCount || counter_idle < 0) {
            direction_idle = -direction_idle;
            counter_idle = direction_idle > 0 ? 0 : ledCount - 1;
        }

        for (int i = 0; i < ledCount; i++) {
            double percent = prevPercent[i] - decay;
            percent = Math.max(percent, 0);
            if (i == counter_idle || i == counter_idle - direction_idle) {
                percent = 1.0;
            }
            prevPercent[i] = percent;
            final double power = (percent * (maxPower - minPower)) + minPower;
            setRGBW(i, (int) power, 0, 0, 0);
        }
        loadData();
        led.start();
    }
}
