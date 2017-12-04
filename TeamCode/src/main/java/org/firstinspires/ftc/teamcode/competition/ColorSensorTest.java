package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by ariporad on 2017-12-02.
 */

@TeleOp(name = "CS TEST")
public class ColorSensorTest extends OpMode {
    ColorSensor cs;

    @Override
    public void init() {
        cs = hardwareMap.colorSensor.get("color_sensor");
    }

    @Override
    public void loop() {
        telemetry.addData("r", cs.red());
        telemetry.addData("g", cs.green());
        telemetry.addData("b", cs.blue());

    }
}
