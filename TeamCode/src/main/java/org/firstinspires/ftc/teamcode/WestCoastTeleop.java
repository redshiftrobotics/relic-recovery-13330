package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Duncan on 9/19/2017.
 */

public class WestCoastTeleop extends OpMode {

    DcMotor leftDrive;
    DcMotor rightDrive;

    @Override
    public void init() {
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
        //leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        drive();
    }

    private void drive(){
        leftDrive.setPower(gamepad1.right_stick_y+gamepad1.right_stick_x);
        rightDrive.setPower(gamepad1.right_stick_y-gamepad1.right_stick_x);
    }
}
