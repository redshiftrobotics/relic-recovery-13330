package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.competition.auto.PulsarAuto;
import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;
import org.redshiftrobotics.lib.blockplacer.Col;

@TeleOp(name = "Color Sensor Test", group = "Debug")
public class ColorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PulsarRobotHardware hw = new PulsarRobotHardware(this, PulsarAuto.Alliance.BLUE);

        waitForStart();

        while (opModeIsActive()) {
            printValues("glyph:", hw.colorSensors.glyph);
            printValues("leftTape:", hw.colorSensors.leftTape);
            printValues("rightTape:", hw.colorSensors.rightTape);
            telemetry.update();
            idle();
        }
    }

    private void printValues(String label, ColorSensor cs) {
        telemetry.addData(label + "as red", PulsarAuto.Alliance.RED.detectTape(cs));
        telemetry.addData(label + "as blue", PulsarAuto.Alliance.BLUE.detectTape(cs));
        telemetry.addData(label + "red", cs.red());
        telemetry.addData(label + "green", cs.green());
        telemetry.addData(label + "blue", cs.blue());
    }
}
