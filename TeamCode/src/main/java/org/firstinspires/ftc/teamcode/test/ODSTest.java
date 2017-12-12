package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;

/**
 * Created by ariporad on 2017-12-05.
 */

public class ODSTest extends OpMode {
    private PulsarRobotHardware hw;

    @Override
    public void init() {
        hw = new PulsarRobotHardware(hardwareMap, null);
        telemetry.addLine("Welcome to the ODS Tester!");
        telemetry.addLine("Press START to begin.");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addLine("ODS Tester");
        //telemetry.addData("distance (cm)", hw.ods.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
