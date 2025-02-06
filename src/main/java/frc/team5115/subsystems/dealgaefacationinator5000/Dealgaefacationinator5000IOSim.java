package frc.team5115.subsystems.dealgaefacationinator5000;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.team5115.Constants;

public class Dealgaefacationinator5000IOSim implements Dealgaefacationinator5000IO {
    private final DoubleSolenoidSim extenderSim;
    private final DCMotorSim motorSim;
    private double motorVolts;
    private boolean state = false;

    public Dealgaefacationinator5000IOSim() {
        final DCMotor motor = DCMotor.getNEO(1);
        motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.0002, 1.0), motor);

        extenderSim = new DoubleSolenoidSim(PneumaticsModuleType.REVPH, 0, 0);
    }

    @Override
    public void updateInputs(Dealgaefacationinator5000IOInputs inputs){
        motorSim.update(Constants.LOOP_PERIOD_SECS);
        inputs.motorVelocityRPM = motorSim.getAngularVelocityRPM();
        inputs.motorVolts = motorVolts;
        inputs.motorAmps = Math.abs(motorSim.getCurrentDrawAmps());

        inputs.state = state;
    }

    @Override
    public void setVoltage(double volts) {
        motorVolts = volts;
        motorSim.setInputVoltage(volts);
    }

    @Override
    public void setPneumatic(boolean extend) {
        state = extend;
        extenderSim.set((extend ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse));
    }

    @Override
    public void setPercent(double percent) {
        motorVolts = MathUtil.clamp(percent*12, +12, -12);
        motorSim.setInput(percent);
    }
}
