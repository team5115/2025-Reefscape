package frc.team5115.subsystems.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.team5115.Constants;

public class ClimberIORev implements ClimberIO {

    private final DigitalInput climbSensor;
    private final DoubleSolenoid m_doubleSolenoid; // Declare as a class-level variable
    private final PneumaticHub m_pH;
    public ClimberIORev() {
        climbSensor = new DigitalInput(Constants.CLIMBER_SENSOR_ID);
        m_pH = new PneumaticHub(Constants.PNEUMATIC_HUB_ID);
        m_doubleSolenoid = m_pH.makeDoubleSolenoid(Constants.CLIMB_FORWARD_CHANNEL, Constants.CLIMB_REVERSE_CHANNEL);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.cageIntake = climbSensor.get();
    }
    
    @Override
    public void extendSolenoid() {
        m_doubleSolenoid.set(Value.kForward);
    }

    @Override
    public void retractSolenoid() {
        m_doubleSolenoid.set(Value.kReverse);
    }

    @Override
    public void stopSolenoid() {
        m_doubleSolenoid.set(Value.kOff);
    }
}