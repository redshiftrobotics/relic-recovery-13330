package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Duncan on 1/20/2018.
 */

@TeleOp(name = "Duncan's dumb test")
public class ServoAllTest extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()){
            hardwareMap
                    .getAll(DcMotor.class)
                    .get( Math.round((gamepad1.left_stick_y+1)*hardwareMap
                            .getAll(DcMotor.class)
                            .size()/2))
                    .setPower(gamepad1.right_stick_y);
        }
    }
}
