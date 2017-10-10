package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Duncan on 10/3/2017.
 */
@Autonomous(name="TheAlgorithm")
public class TheAlgorithm extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    float xPos;
    float yPos;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get("m0");
        backLeft = hardwareMap.dcMotor.get("m1");
        frontRight = hardwareMap.dcMotor.get("m2");
        backRight = hardwareMap.dcMotor.get("m3");

        waitForStart();

        while(opModeIsActive()){
            xPos = frontLeft.getCurrentPosition();
            yPos = frontRight.getCurrentPosition();
            telemetry.addData("Position", "(" + xPos + ", " + yPos + ")");
            telemetry.update();
        }
    }
}
