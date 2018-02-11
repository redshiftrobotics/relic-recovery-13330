package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Jewel Kick Test")
public class JewelKickerTest extends OpMode {
    Servo l;
    Servo r;

    @Override
    public void init() {
        l = hardwareMap.servo.get("r2s3");
        r = hardwareMap.servo.get("r2s4");
    }

    @Override
    public void loop() {
        l.setPosition(Math.abs(gamepad1.left_stick_x));
        r.setPosition(Math.abs(gamepad1.right_stick_x));
    }
}
