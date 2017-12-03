package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ariporad on 2017-12-02.
 */

public class PulsarRobotHardware {
    public final DcMotor frontLeft;
    public final DcMotor frontRight;
    public final DcMotor backLeft;
    public final DcMotor backRight;

    public final Servo leftJewelServo;
    public final Servo rightJewelServo;

    //public final ColorSensor leftJewelDetector;
    //public final ColorSensor rightJewelDetector;

    public final DcMotor conveyor;
    public final Servo conveyorLift;

    public final Servo intakeServoLeft;
    public final Servo intakeServoRight;
    public final DcMotor leftIntake;
    public final DcMotor rightIntake;

    public final BNO055IMU imu;

    public PulsarRobotHardware(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        leftJewelServo = hardwareMap.servo.get("leftJewel");
        rightJewelServo = hardwareMap.servo.get("rightJewel");
        //leftJewelDetector = hardwareMap.colorSensor.get("leftCS");
        //rightJewelDetector = hardwareMap.colorSensor.get("rightCS");

        leftIntake = hardwareMap.dcMotor.get("collectorLeft");
        rightIntake = hardwareMap.dcMotor.get("collectorRight");
        intakeServoLeft = hardwareMap.servo.get("collectorServoLeft");
        intakeServoRight = hardwareMap.servo.get("collectorServoRight");

        conveyor = hardwareMap.dcMotor.get("conveyor");
        conveyorLift = hardwareMap.servo.get("conveyorLift");
    }

    public void initializePositions() {
        intakeServoRight.setDirection(Servo.Direction.REVERSE);
        leftJewelServo.setDirection(Servo.Direction.REVERSE);
        rightJewelServo.setDirection(Servo.Direction.REVERSE);

        intakeServoLeft.setPosition(0.8);
        intakeServoRight.setPosition(0.85);

        conveyorLift.setPosition(0.45);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        leftJewelServo.setPosition(0.3);
        rightJewelServo.setPosition(0.9);
    }

    public void initializePositionsTeleop() {
        intakeServoRight.setDirection(Servo.Direction.REVERSE);
        leftJewelServo.setDirection(Servo.Direction.REVERSE);
        rightJewelServo.setDirection(Servo.Direction.REVERSE);

        leftJewelServo.setPosition(0.35);
        rightJewelServo.setPosition(0.8);
    }
}
