package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.redshiftrobotics.lib.descartes.UltrasonicDistanceSensor;

/**
 * Created by ariporad on 2017-10-24.
 */

@TeleOp(name = "Ultra Test")
public class UltraTest extends OpMode {
    UltrasonicDistanceSensor ultra;

    @Override
    public void init() {
        ultra = hardwareMap.get(UltrasonicDistanceSensor.class, "ultra");
        ultra.startReading();
    }

    @Override
    public void loop() {
        telemetry.addData("cm", ultra.getDistance(DistanceUnit.CM, telemetry));
        telemetry.update();
    }
}
