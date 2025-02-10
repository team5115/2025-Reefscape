package frc.team5115.subsystems.bling;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Bling extends SubsystemBase {
    public static final int LED_COUNT = 144; // ! must be divisble by 3
    public static final int LED_PORT = 0; // pwm

    private final BlingIO io;
    private final BlingIOInputsAutoLogged inputs = new BlingIOInputsAutoLogged();

    private final int period = 1;
    private final double tailLength = 20;
    private final int minPower = 10;
    private final int maxPower = 255;
    private final double decay = (maxPower - minPower) / 1d / tailLength;

    private int timer = 0;
    private int counter = 0;
    private int direction = 1;
    private int counter2 = 0;

    public Bling(BlingIO io) {
        this.io = io;
        io.start();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Bling/LedStrip", inputs.ledStrip);
        Logger.recordOutput("Bling/Timer", timer);
        Logger.recordOutput("Bling/Counter", counter);
        Logger.recordOutput("Bling/Direction", direction);
    }

    /** Set the whole strip to off */
    public Command off() {
        return staticColor(0, 0, 0, 0);
    }

    /** The classic red scrolling KITT pattern */
    public Command redKITT() {
        return scrollingKITT(1, 0, 0, 0);
    }

    /** The KITT pattern but green */
    public Command greenKITT() {
        return scrollingKITT(0, 1, 0, 0);
    }

    public Command yellowKITT() {
        return scrollingKITT(1, 1, 0, 0);
    }

    public Command blueKITT() {
        return scrollingKITT(0, 0, 1, 0);
    }

    public Command blueSolid() {
        return staticColor(0, 0, 1, 0);
    }

    public Command purpleSolid() {
        return staticColor(0.35f, 0, 0.75f, 0);
    }

    public Command yellowScrollIn() {
        return scrollIn(1, 1, 0, 0);
    }

    public Command greenScrollIn() {
        return scrollIn(0, 1, 0, 0);
    }

    public Command blueScrollIn() {
        return scrollIn(0, 0, 1, 0);
    }

    /**
     * Set the whole strip to a color
     *
     * @param red the red component (0-255)
     * @param green the green component (0-255)
     * @param blue the blue component (0-255)
     * @param white the white component (0-255)
     * @return a command that sets the color
     */
    public Command staticColor(float red, float green, float blue, float white) {
        return Commands.startRun(
                () -> {},
                () -> {
                    for (int i = 0; i < LED_COUNT; i++) {
                        io.setRGBW(
                                i, (int) (red * 255), (int) (green * 255), (int) (blue * 255), (int) (white * 255));
                    }
                },
                this);
    }

    /**
     * The classic KITT light pattern using the color percentages passed in
     *
     * @param red [0,1]
     * @param green [0,1]
     * @param blue [0,1]
     * @param white [0,1]
     * @return a command that runs the pattern
     */
    public Command scrollingKITT(double red, double green, double blue, double white) {
        return Commands.startRun(
                () -> {
                    // Start
                    for (int i = 0; i < LED_COUNT; i++) {
                        io.setRGBW(i, 0, 0, 0, 0);
                    }
                },
                () -> {
                    // Repeating
                    timer++;
                    if (timer >= period) {
                        timer = 0;
                    } else {
                        return;
                    }

                    counter += direction;
                    if (counter >= LED_COUNT || counter < 0) {
                        direction = -direction;
                        counter = direction > 0 ? 0 : LED_COUNT - 1;
                    }

                    for (int i = 0; i < LED_COUNT; i++) {
                        double power;
                        if (i == counter) {
                            power = maxPower;
                        } else {
                            power =
                                    Math.max(
                                            minPower,
                                            totalBrightness(inputs.ledStrip[i]) / (red + green + blue + white) - decay);
                        }
                        io.setRGBW(
                                i,
                                (int) (power * red),
                                (int) (power * green),
                                (int) (power * blue),
                                (int) (power * white));
                    }
                },
                this);
    }

    public Command scrollIn(double red, double green, double blue, double white) {
        return Commands.startRun(
                () -> {
                    // Start
                    for (int i = 0; i < LED_COUNT; i++) {
                        io.setRGBW(i, 0, 0, 0, 0);
                    }
                    counter2 = LED_COUNT;
                    counter = 0;
                },
                () -> {
                    // Repeating
                    timer++;
                    if (timer >= period) {
                        timer = 0;
                    } else {
                        return;
                    }

                    counter += 1;
                    counter2 += -1;
                    if (counter >= LED_COUNT - LED_COUNT / 2) {
                        counter = 0;
                        counter2 = LED_COUNT;
                    }

                    for (int i = 0; i < LED_COUNT; i++) {
                        double power;
                        if (i == counter || i == counter2) {
                            power = maxPower;
                        } else {
                            power =
                                    Math.max(
                                            minPower,
                                            totalBrightness(inputs.ledStrip[i]) / (red + green + blue + white) - decay);
                        }
                        io.setRGBW(
                                i,
                                (int) (power * red),
                                (int) (power * green),
                                (int) (power * blue),
                                (int) (power * white));
                    }
                },
                this);
    }

    private double totalBrightness(int[] color) {
        return color[0] + color[1] + color[2] + color[3];
    }
}
