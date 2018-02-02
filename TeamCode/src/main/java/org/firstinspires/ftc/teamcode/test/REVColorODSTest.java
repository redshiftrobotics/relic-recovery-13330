package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="REV Test")
public class REVColorODSTest extends OpMode {
    ColorSensor cs;
    DistanceSensor ds;

    @Override
    public void init() {
        cs = hardwareMap.get(ColorSensor.class, "r2c3");
        ds = hardwareMap.get(DistanceSensor.class, "r2c3");
    }

    @Override
    public void loop() {
        telemetry.addData("R", cs.red());
        telemetry.addData("G", cs.green());
        telemetry.addData("B", cs.blue());
        telemetry.addData("Alpha", cs.alpha());
        telemetry.addData("CM", ds.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
