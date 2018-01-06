package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Color Sensor Test", group = "Debug")
public class ColorSensorTest extends OpMode {
    ColorSensor cs;

    @Override
    public void init() {
        telemetry.addLine("Welcome to the Color Sensor Tester!");
        telemetry.addLine("This OpMode Requires a Color Sensor to be named \"color_sensor\" in the config.");
        telemetry.addLine();
        telemetry.addLine("Press START to Begin.");
        telemetry.update();

        cs = hardwareMap.colorSensor.get("color_sensor");
    }

    @Override
    public void loop() {
        telemetry.addLine("Color Values:");
        telemetry.addData("r", cs.red());
        telemetry.addData("g", cs.green());
        telemetry.addData("b", cs.blue());
    }
}
