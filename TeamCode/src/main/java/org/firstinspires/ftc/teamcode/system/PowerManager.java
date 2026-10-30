package org.firstinspires.ftc.teamcode.system;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

public final class PowerManager {

    private final VoltageSensor voltageSensor;
    private double currentVoltage = RobotConstants.NOMINAL_VOLTAGE;

    public PowerManager(VoltageSensor voltageSensor) {
        this.voltageSensor = voltageSensor;
    }

    public void update() {
        if (voltageSensor != null) {
            currentVoltage = voltageSensor.getVoltage();
        }
    }

    public double getVoltage() {
        return currentVoltage;
    }

    public double scaleOutput(double demandedPower) {
        if (currentVoltage < RobotConstants.VOLTAGE_CLAMP_THRESHOLD) {
            return demandedPower;
        }
        if (currentVoltage >= RobotConstants.NOMINAL_VOLTAGE) {
            return demandedPower;
        }
        double compensationVoltage = Math.max(RobotConstants.MIN_COMPENSATION_VOLTAGE, currentVoltage);
        return demandedPower * (RobotConstants.NOMINAL_VOLTAGE / compensationVoltage);
    }

    public double getDrivetrainScale() {
        if (currentVoltage < RobotConstants.LOAD_SHED_THRESHOLD) {
            return 0.50;
        } else if (currentVoltage < RobotConstants.VOLTAGE_CLAMP_THRESHOLD) {
            return 0.75;
        }
        return 1.0;
    }

    public boolean shouldShedAuxiliaryLoads() {
        return currentVoltage < RobotConstants.LOAD_SHED_THRESHOLD;
    }
}
