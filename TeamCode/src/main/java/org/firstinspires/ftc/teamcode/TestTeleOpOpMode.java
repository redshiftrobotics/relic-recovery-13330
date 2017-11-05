package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ariporad on 2017-10-04.
 */

@TeleOp(name = "Test TeleOp")
@Disabled
public class TestTeleOpOpMode extends OpMode {
    DcMotor m0;
    DcMotor m1;
    DcMotor m2;
    DcMotor m3;

    @Override
    public void init() {
        m0 = hardwareMap.dcMotor.get("m0");
        m1 = hardwareMap.dcMotor.get("m1");
        m2 = hardwareMap.dcMotor.get("m2");
        m3 = hardwareMap.dcMotor.get("m3");

    }

    @Override
    public void loop() {
        double left = -gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;
        m0.setPower(left);
        m1.setPower(left);
        m2.setPower(right);
        m3.setPower(right);
    }
}
