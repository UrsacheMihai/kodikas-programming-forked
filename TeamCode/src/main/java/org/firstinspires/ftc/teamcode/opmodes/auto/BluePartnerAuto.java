package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.FieldConfig;

@Autonomous(name = "Blue Partner Auto", group = "BIOBUZZ", preselectTeleOp = "BioBuzz TeleOp")
public final class BluePartnerAuto extends BioBuzzAutoEngine {

    public BluePartnerAuto() {
        super(FieldConfig.Alliance.BLUE, AutoRoutine.PARTNER, FieldConfig.POSE_NUMBER_SCORING);
    }
}
