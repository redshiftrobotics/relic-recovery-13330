package org.firstinspires.ftc.teamcode;

import android.text.InputFilter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Duncan on 9/20/2017.
 */

public class GroundLineChecker extends LinearOpMode{

    ColorSensor cs;
    DcMotor leftDrive;
    DcMotor rightDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
        //leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        cs = hardwareMap.colorSensor.get("cs");

        waitForStart();

        leftDrive.setPower(1.0);
        rightDrive.setPower(1.0);

        while(!CheckLine() && opModeIsActive()){

        }

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);

        Thread.sleep(1000);

        leftDrive.setPower(-1.0);
        rightDrive.setPower(-1.0);

        while(!CheckLine() && opModeIsActive()){

        }

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }

    private boolean CheckLine(){
        return cs.red() > 3 || cs.blue() > 3;
    }
}
