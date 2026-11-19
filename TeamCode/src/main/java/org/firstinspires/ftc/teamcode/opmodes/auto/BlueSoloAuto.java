package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.FieldConfig;

@Autonomous(name = "Blue Solo Auto", group = "BIOBUZZ", preselectTeleOp = "BioBuzz TeleOp")
public final class BlueSoloAuto extends BioBuzzAutoEngine {

    public BlueSoloAuto() {
        super(FieldConfig.Alliance.BLUE, AutoRoutine.SOLO, FieldConfig.POSE_NUMBER_GARDEN);
    }
}
