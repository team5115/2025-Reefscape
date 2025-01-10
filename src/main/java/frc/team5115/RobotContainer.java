package frc.team5115;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SerialPort port;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        port = new SerialPort(115200, SerialPort.Port.kUSB1);
    }

    public void robotPeriodic() {
        final byte[] byteArray = port.read(2);
        Logger.recordOutput("Raw Bytes", byteArray);

        final ByteBuffer bb = ByteBuffer.allocate(2);
        bb.order(ByteOrder.BIG_ENDIAN);
        bb.put(byteArray);
        final short value = bb.getShort(0);
        Logger.recordOutput("Short", value);
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
