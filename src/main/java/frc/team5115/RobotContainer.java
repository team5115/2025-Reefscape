package frc.team5115;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SerialPort port;
    private final ArrayList<Character> queue = new ArrayList<Character>();
    private int distance;
    private boolean hasMeasurement;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        port = new SerialPort(115200, SerialPort.Port.kUSB1);
    }

    public void robotPeriodic() {
        while (port.getBytesReceived() > 0) {
            final char nextChar = (char) port.read(1)[0];
            if (nextChar == '\n') {
                final StringBuilder builder = new StringBuilder();
                queue.forEach((ch) -> builder.append(ch));
                queue.clear();
                final String str = builder.toString().trim();
                int num = -1;
                try {
                    num = Integer.parseInt(str, 10);
                } catch (NumberFormatException e) {
                    System.out.print("ParseInt error: ");
                    System.out.println(e);
                }
                if (hasMeasurement = (num != -1)) {
                    distance = num;
                }
                Logger.recordOutput("String", str);
            } else {
                queue.add(nextChar);
            }
        }
        Logger.recordOutput("QSize", queue.size());
        Logger.recordOutput("Distance (mm)", distance);
        Logger.recordOutput("Has Measurement", hasMeasurement);
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
