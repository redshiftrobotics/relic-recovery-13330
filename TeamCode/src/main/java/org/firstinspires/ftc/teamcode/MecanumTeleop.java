package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Duncan on 9/19/2017.
 */

public class MecanumTeleop extends OpMode{

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        drive();
    }

    private void drive(){
        frontLeft.setPower(gamepad1.right_stick_y-gamepad1.right_stick_x+gamepad1.left_stick_x);
        frontRight.setPower(gamepad1.right_stick_y-gamepad1.right_stick_x-gamepad1.left_stick_x);
        backLeft.setPower(gamepad1.right_stick_y+gamepad1.right_stick_x+gamepad1.left_stick_x);
        backRight.setPower(gamepad1.right_stick_y+gamepad1.right_stick_x-gamepad1.left_stick_x);
    }
}