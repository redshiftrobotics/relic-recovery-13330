package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Duncan on 9/23/2017.
 */

@TeleOp(name="Soccer Bot", group="Outreach")
public class SoccerBot extends OpMode {

    DcMotor leftDrive;
    DcMotor rightDrive;

    @Override
    public void init() {
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
    }

    @Override
    public void loop() {
        leftDrive.setPower(Range.clip((gamepad1.right_stick_y*0.7)+(-gamepad1.right_stick_x*0.3),-1.0,1.0));
        rightDrive.setPower(Range.clip((gamepad1.right_stick_y*0.7)+(gamepad1.right_stick_x*0.3),-1.0,1.0));
    }
}
