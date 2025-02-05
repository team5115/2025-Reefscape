package frc.team5115.subsystems.dealgaefacationinator5000;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;

public class Dealgaefacationinator5000IOSim implements Dealgaefacationinator5000IO {
    public SolenoidSim extenderSim;
    public DCMotorSim motorSim;

    public Dealgaefacationinator5000IOSim() {
        final DCMotor motor = DCMotor.getNEO(1);
        motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.0002, 1.0), motor);

        extenderSim = new SolenoidSim(PneumaticsModuleType.REVPH, 0);
    }

    @Override
    public void setVoltage(double volts) {
        motorSim.setInputVoltage(volts);
    }

    @Override
    public void setPneumatic(boolean extend) {
        extenderSim.setOutput(extend);
    }

    @Override
    public void setPercent(double percent) {
        motorSim.setInput(percent);
    }
}
