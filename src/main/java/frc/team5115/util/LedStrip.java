package frc.team5115.util;

import edu.wpi.first.hal.AddressableLEDJNI;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.PWMJNI;

public class LedStrip implements AutoCloseable {
    private final int m_pwmHandle;
    private final int m_handle;

    private final byte[] buffer;
    /**
     * Constructs a new driver for a specific port.
     *
     * @param port the output port to use (Must be a PWM header, not on MXP)
     */
    public LedStrip(int port, int length) {
        m_pwmHandle = PWMJNI.initializePWMPort(HAL.getPort((byte) port));
        m_handle = AddressableLEDJNI.initialize(m_pwmHandle);
        buffer = new byte[length * 5];
        HAL.report(tResourceType.kResourceType_AddressableLEDs, port + 1);
    }

    @Override
    public void close() {
        if (m_handle != 0) {
            AddressableLEDJNI.free(m_handle);
        }
        if (m_pwmHandle != 0) {
            PWMJNI.freePWMPort(m_pwmHandle);
        }
    }

    /**
     * Sets the length of the LED strip.
     *
     * <p>Calling this is an expensive call, so it's best to call it once, then just update data.
     *
     * <p>The max length is 5460 LEDs.
     *
     * @param length the strip length
     */
    public void setLength(int length) {
        AddressableLEDJNI.setLength(m_handle, length);
    }

    /**
     * Sets the LED output data.
     *
     * <p>If the output is enabled, this will start writing the next data cycle. It is safe to call,
     * even while output is enabled.
     *
     * @param buffer the buffer to write
     */
    public void setData() {
        AddressableLEDJNI.setData(m_handle, this.buffer);
    }

    /**
     * Sets the bit timing.
     *
     * <p>By default, the driver is set up to drive WS2812Bs, so nothing needs to be set for those.
     *
     * @param highTime0NanoSeconds high time for 0 bit (default 400ns)
     * @param lowTime0NanoSeconds low time for 0 bit (default 900ns)
     * @param highTime1NanoSeconds high time for 1 bit (default 900ns)
     * @param lowTime1NanoSeconds low time for 1 bit (default 600ns)
     */
    public void setBitTiming(
            int highTime0NanoSeconds,
            int lowTime0NanoSeconds,
            int highTime1NanoSeconds,
            int lowTime1NanoSeconds) {
        AddressableLEDJNI.setBitTiming(
                m_handle,
                highTime0NanoSeconds,
                lowTime0NanoSeconds,
                highTime1NanoSeconds,
                lowTime1NanoSeconds);
    }

    /**
     * Sets the sync time.
     *
     * <p>The sync time is the time to hold output so LEDs enable. Default set for WS2812B.
     *
     * @param syncTimeMicroSeconds the sync time (default 280us)
     */
    public void setSyncTime(int syncTimeMicroSeconds) {
        AddressableLEDJNI.setSyncTime(m_handle, syncTimeMicroSeconds);
    }

    /**
     * Starts the output.
     *
     * <p>The output writes continuously.
     */
    public void start() {
        AddressableLEDJNI.start(m_handle);
    }

    /** Stops the output. */
    public void stop() {
        AddressableLEDJNI.stop(m_handle);
    }

    public void setRGB(int index, int r, int g, int b, int w) {
        buffer[index * 5] = (byte) b;
        buffer[(index * 5) + 1] = (byte) g;
        buffer[(index * 5) + 2] = (byte) r;
        buffer[(index * 5) + 3] = (byte) w;
        buffer[(index * 5) + 4] = (byte) 0;
    }
}
