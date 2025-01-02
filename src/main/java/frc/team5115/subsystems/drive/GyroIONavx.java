package frc.team5115.subsystems.drive;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIONavx implements GyroIO {
    private final AHRS ahrs = new AHRS(AHRS.NavXComType.kMXP_SPI);

    public GyroIONavx() {
        checkForConnection();
        ahrs.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = ahrs.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(-ahrs.getYaw());
    }

    /**
     * Checks to see if the navx is connected. Prints a message to the console with the result.
     *
     * @return Whether or not the navx is connected
     */
    public boolean checkForConnection() {
        if (ahrs.isConnected()) {
            System.out.println("NavX is connected");
            return true;
        }
        System.out.println("NavX is NOT connected!!!");
        return false;
    }
}
