package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ariporad on 2017-11-25.
 */

@TeleOp(name = "Encoder Log")
public class EncoderLog extends OpMode {
    DcMotor m0;
    DcMotor m1;
    DcMotor m2;
    DcMotor m3;

    @Override
    public void init() {
        m0 = hardwareMap.dcMotor.get("fl");
        m1 = hardwareMap.dcMotor.get("fr");
        m2 = hardwareMap.dcMotor.get("bl");
        m3 = hardwareMap.dcMotor.get("br");
    }

    @Override
    public void loop() {
        telemetry.addData("fl", m0.getCurrentPosition());
        telemetry.addData("fr", m1.getCurrentPosition());
        telemetry.addData("bl", m2.getCurrentPosition());
        telemetry.addData("br", m3.getCurrentPosition());
        telemetry.update();
    }
}
