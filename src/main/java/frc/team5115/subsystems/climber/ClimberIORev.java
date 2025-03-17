package frc.team5115.subsystems.climber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.team5115.Constants;
import edu.wpi.first.wpilibj.PWM;

public class ClimberIORev implements ClimberIO {

    private final DigitalInput climbSensor;
    private final DoubleSolenoid m_doubleSolenoid; // Declare as a class-level variable
    private DigitalOutput actuator;
    // private PWM linearActuator;
    private boolean block = false;

    public ClimberIORev(PneumaticHub hub) {
        actuator = new DigitalOutput(Constants.BLOCK_ACTUATOR_ID);
        // linearActuator = new PWM(Constants.BLOCK_ACTUATOR_ID);
        climbSensor = new DigitalInput(Constants.CLIMB_INAKE_SENSOR);
        m_doubleSolenoid =
                hub.makeDoubleSolenoid(Constants.CLIMB_FORWARD_CHANNEL, Constants.CLIMB_REVERSE_CHANNEL);
        actuator.set(true);
        block = true;
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.cageIntake = !climbSensor.get();
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

    @Override
    public void toggleBlock() {
        block = !block;
        actuator.set(block);
    }
}
