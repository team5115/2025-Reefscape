package frc.team5115.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.team5115.Constants;

public class ClimberIOSim implements ClimberIO {
    private final DCMotorSim leftSim;
    private final DCMotorSim rightSim;
    private double leftAppliedVolts;
    private double rightAppliedVolts;

    public ClimberIOSim() {
        final DCMotor motor = DCMotor.getNEO(1);
        leftSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.005, 0.0625), motor);
        rightSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.005, 0.0625), motor);
    }

    @Override
    public void setLeftPercent(double speed) {
        leftAppliedVolts = MathUtil.clamp(speed * 12, -12.0, +12.0);
        leftSim.setInputVoltage(leftAppliedVolts);
    }

    @Override
    public void setRightPercent(double speed) {
        rightAppliedVolts = MathUtil.clamp(speed * 12, -12.0, +12.0);
        rightSim.setInputVoltage(rightAppliedVolts);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        leftSim.update(Constants.LOOP_PERIOD_SECS);
        rightSim.update(Constants.LOOP_PERIOD_SECS);
        inputs.leftAngle = Rotation2d.fromRotations(leftSim.getAngularPositionRotations());
        inputs.rightAngle = Rotation2d.fromRotations(rightSim.getAngularPositionRotations());
        inputs.leftVelocityRPM = leftSim.getAngularVelocityRPM();
        inputs.rightVelocityRPM = rightSim.getAngularVelocityRPM();
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.leftCurrentAmps = Math.abs(leftSim.getCurrentDrawAmps());
        inputs.rightCurrentAmps = Math.abs(rightSim.getCurrentDrawAmps());
    }
}
