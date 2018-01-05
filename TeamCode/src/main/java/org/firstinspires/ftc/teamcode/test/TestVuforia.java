package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.VuforiaController;
import org.redshiftrobotics.lib.vuforia.ColumnController;

/**
 * Created by ariporad on 2017-12-16.
 */

@Autonomous(name="TestVuforia")
public class TestVuforia extends OpMode {
    private VuforiaController vc;

    @Override
    public void init() {
        vc = new VuforiaController(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("col", vc.detectColumn().toString());
        telemetry.update();
    }
}
