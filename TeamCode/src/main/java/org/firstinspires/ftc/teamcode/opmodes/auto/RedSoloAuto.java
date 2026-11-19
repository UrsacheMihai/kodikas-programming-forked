package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.FieldConfig;

@Autonomous(name = "Red Solo Auto", group = "BIOBUZZ", preselectTeleOp = "BioBuzz TeleOp")
public final class RedSoloAuto extends BioBuzzAutoEngine {

    public RedSoloAuto() {
        super(FieldConfig.Alliance.RED, AutoRoutine.SOLO, FieldConfig.POSE_NUMBER_GARDEN);
    }
}
