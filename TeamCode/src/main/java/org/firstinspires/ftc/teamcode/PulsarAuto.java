package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;



public class PulsarAuto extends LinearOpMode {
    enum StartPosition {
        BACK,
        FRONT

       /* int value;

        StartPosition(int v) {
            this.value =v;
        }*/
    }

    enum Alliance {
        BLUE, RED;

        public double getJewelUpPosition(StartPosition pos) { return this == BLUE ? 0 : 0; }
        public double getJewelDownPosition(StartPosition pos) { return this == BLUE ? 0 : 0; }

    }




    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    Servo jewel;

    Servo leftJewel;
    Servo rightJewel;

    ColorSensor leftJewelDetector;
    ColorSensor rightJewelDetector;

    StartPosition startPosition = StartPosition.FRONT;
    Alliance alliance = Alliance.BLUE;

    MecanumRobot robot;

    ColumnController columnController;

    BNO055IMU imu;

    RelicRecoveryVuMark column;



    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        columnController = new ColumnController(hardwareMap);
        leftJewel = hardwareMap.servo.get("leftJewel");
        rightJewel = hardwareMap.servo.get("rightJewel");

        leftJewel.setPosition(Alliance.BLUE.getJewelUpPosition(startPosition));
        rightJewel.setPosition(Alliance.RED.getJewelUpPosition(startPosition));

        jewel = (alliance == Alliance.BLUE) ? leftJewel : rightJewel;


        robot = new MecanumRobot(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                imu,
                this,
                telemetry
        );
        waitForStart();
        column = columnController.getColumn();

        jewel.setPosition(alliance.getJewelDownPosition(startPosition));











    }
}
