package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * Created by adam on 10/14/17.
 */

@TeleOp(name = "cs_test")
public class ColorSensorTest extends OpMode {

    ColorSensor cs;

    @Override public void init() {
        cs = hardwareMap.colorSensor.get("cs");
    }

    @Override public void loop() {
        telemetry.addData("Current Value: ", "Red: " + cs.red() + " Blue: " + cs.blue());
        telemetry.update();
    }
}
