package frc.team5115.subsystems.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import org.littletonrobotics.junction.Logger;

public class ClimberIOSim implements ClimberIO {
    private Value pistonState;

    public ClimberIOSim() {}

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.cageIntake = true;
        Logger.recordOutput("PISTON STATE", pistonState);
    }

    @Override
    public void extendSolenoid() {
        pistonState = Value.kForward;
    }

    @Override
    public void retractSolenoid() {
        pistonState = Value.kReverse;
    }

    @Override
    public void stopSolenoid() {
        pistonState = Value.kOff;
    }
}
