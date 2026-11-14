package org.firstinspires.ftc.teamcode.config;

public final class HardwarePorts {

    public static final String FL_MOTOR = "FLMotor";
    public static final String FR_MOTOR = "FRMotor";
    public static final String BL_MOTOR = "BLMotor";
    public static final String BR_MOTOR = "BRMotor";
    public static final String TRANSFER_MOTOR = "TransferMotor";
    public static final String OUTTAKE_MOTOR = "OuttakeMotor";
    public static final String L_TURRET_MOTOR = "LTurretMotor";
    public static final String R_TURRET_MOTOR = "RTurretMotor";
    public static final String PLOW_SERVO = "PlowServo";
    public static final String TURRET_SERVO = "TurretServo";
    public static final String HOOD_SERVO = "HoodServo";
    public static final String LIMELIGHT_SERVO = "LimelightServo";
    public static final String PINPOINT = "pinpoint";
    public static final String LIMELIGHT = "limelight";
    public static final String COLOR_SENSOR = "colorSensor";

    public static final int PORT_COUNT = 15;

    public static final String[] ALL_PORTS = new String[] {
            FL_MOTOR,
            FR_MOTOR,
            BL_MOTOR,
            BR_MOTOR,
            TRANSFER_MOTOR,
            OUTTAKE_MOTOR,
            L_TURRET_MOTOR,
            R_TURRET_MOTOR,
            PLOW_SERVO,
            TURRET_SERVO,
            HOOD_SERVO,
            LIMELIGHT_SERVO,
            PINPOINT,
            LIMELIGHT,
            COLOR_SENSOR
    };

    private HardwarePorts() {
    }
}
