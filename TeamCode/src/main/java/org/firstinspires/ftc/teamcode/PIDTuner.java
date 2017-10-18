package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Duncan on 10/14/2017.
 */
@TeleOp(name="PID Tuner")
public class PIDTuner extends LinearOpMode {

    BNO055IMU imu;

    long lastTime;

    DcMotor frontLeft, frontRight, backLeft, backRight;
    MecanumRobot robot;


    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");

        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        robot = new MecanumRobot(frontLeft,
                frontRight,
                backLeft,
                backRight,
                imu, null, this, telemetry);
        robot.imupidController.setTuning(0, 0, 0);



        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("P: " + robot.imupidController.pConst + " I: " + robot.imupidController.iConst + " D: " + robot.imupidController.dConst, "");
            telemetry.update();
            robot.imupidController.pConst += gamepad1.left_stick_y * 0.0001; //(System.currentTimeMillis() - lastTime);
            robot.imupidController.iConst += gamepad1.right_stick_y * 0.0001; //(System.currentTimeMillis() - lastTime);
            if(gamepad1.dpad_down) {
                robot.imupidController.dConst += 0.0001;//(System.currentTimeMillis() - lastTime);
            }else if(gamepad1.dpad_up) {
                robot.imupidController.dConst -= 0.0001;//(System.currentTimeMillis() - lastTime);
            }
            if(gamepad1.a){
                robot.MoveStraight(0.5f, Math.PI/2, 8000);
                robot.STOP();
            }
            lastTime = System.currentTimeMillis();

        }
    }
}
