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

    boolean lastPressedB = false, lastPressedX = false, lastPressedY = false;

    static double INCREMENT = 0.5;
    static long STRAIGHT_SECONDS = 5000;

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

        telemetry.addLine("PID tuning opmode.\n Press B to increment P, X " +
                "to increment I, and Y to increment D. Use the up DPAD Arrow + bound button decreases P, I, or D ");
        telemetry.update();

        while (opModeIsActive()) {
            telemetry.addData("P: " + robot.imupidController.pConst +
                    " I: " + robot.imupidController.iConst +
                    " D: " + robot.imupidController.dConst,
                    ""
            );

            telemetry.update();
            if (gamepad1.b && !lastPressedB) {
                if (gamepad1.dpad_up) {
                    robot.imupidController.pConst -= INCREMENT;
                } else {
                    robot.imupidController.pConst += INCREMENT;
                }
                lastPressedB = true;
            } else if (!gamepad1.a) {
                lastPressedB = false;
            }

            if (gamepad1.x && !lastPressedX) {
                if (gamepad1.dpad_up) {
                    robot.imupidController.iConst -= INCREMENT;
                } else {
                    robot.imupidController.iConst += INCREMENT;
                }
                lastPressedX = true;
            } else if (!gamepad1.b) {
                lastPressedX = false;
            }

           if (gamepad1.y && !lastPressedY) {
               if (gamepad1.dpad_up) {
                   robot.imupidController.dConst -= INCREMENT;
               } else {
                   robot.imupidController.dConst += INCREMENT;
               }
               lastPressedY = true;
           } else if (!gamepad1.b) {
               lastPressedY = false;
           }

            if(gamepad1.a){
                robot.MoveStraight(0.5f, Math.PI/2, STRAIGHT_SECONDS);
                robot.STOP();
            }
        }
    }
}
