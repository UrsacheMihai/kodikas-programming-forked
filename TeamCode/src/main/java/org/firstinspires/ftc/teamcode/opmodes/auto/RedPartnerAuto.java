package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.FieldConfig;

@Autonomous(name = "Red Partner Auto", group = "BIOBUZZ", preselectTeleOp = "BioBuzz TeleOp")
public final class RedPartnerAuto extends BioBuzzAutoEngine {

    public RedPartnerAuto() {
        super(FieldConfig.Alliance.RED, AutoRoutine.PARTNER, FieldConfig.POSE_NUMBER_SCORING);
    }
}
