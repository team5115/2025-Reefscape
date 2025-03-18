package frc.team5115.subsystems.climber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.team5115.Constants;

public class ClimberIORev implements ClimberIO {

    private final DigitalInput climbSensor;
    private final DoubleSolenoid m_doubleSolenoid;
    private PWM linearActuator;
    private boolean block = false;

    public ClimberIORev(PneumaticHub hub) {
        linearActuator = new PWM(Constants.BLOCK_ACTUATOR_ID);
        linearActuator.setPosition(1.0); // set to block
        block = true;
        climbSensor = new DigitalInput(Constants.CLIMB_INAKE_SENSOR);
        m_doubleSolenoid =
                hub.makeDoubleSolenoid(Constants.CLIMB_FORWARD_CHANNEL, Constants.CLIMB_REVERSE_CHANNEL);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.cageIntake = !climbSensor.get();
    }

    @Override
    public void toggleShield() {
        block = !block;
        if (block) {
            linearActuator.setPosition(1.0);
        } else {
            linearActuator.setPosition(0.0);
        }
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
