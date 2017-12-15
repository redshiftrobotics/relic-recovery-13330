package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.competition.PulsarAuto;

/**
 * Created by ariporad on 2017-12-02.
 */

public class PulsarRobotHardware {
    public PulsarAuto.Alliance alliance;

    // Sensors
    public final DcMotor frontLeft;
    public final DcMotor frontRight;
    public final DcMotor backLeft;
    public final DcMotor backRight;

    public final BNO055IMU imu;

    public final Servo leftJewelServo;
    public final Servo rightJewelServo;
    public final Servo jewelServo;

    public final ColorSensor leftJewelDetector;
    public final ColorSensor rightJewelDetector;
    public final ColorSensor jewelDetector;

    public final DcMotor conveyor;
    public final Servo conveyorLiftLeft;
    public final Servo conveyorLiftRight;

    public final Servo intakeServoLeft;
    public final Servo intakeServoRight;
    public final DcMotor leftIntake;
    public final DcMotor rightIntake;

    // Constants
    public final double LEFT_JEWEL_UP_POSITON = 0.35;
    public final double LEFT_JEWEL_DOWN_POSITON = 0;
    public final double RIGHT_JEWEL_UP_POSITON = 0.8;
    public final double RIGHT_JEWEL_DOWN_POSITON = 0;

    public final double CONVEYOR_SPEED = 0.65;
    public final double CONVEYOR_LIFT_UP_POSITION = 0.45;
    public final double CONVEYOR_LIFT_DOWN_POSITON = 0.28;

    public final double INTAKE_UP_POSITION = 1;
    public final double INTAKE_DOWN_POSITION = 0;


    public PulsarRobotHardware(HardwareMap hardwareMap, PulsarAuto.Alliance alliance) {
        this.alliance = alliance;

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initalizeIMU(imu);

        leftJewelServo = hardwareMap.servo.get("leftJewel");
        rightJewelServo = hardwareMap.servo.get("rightJewel");
        jewelServo = alliance == PulsarAuto.Alliance.BLUE ? leftJewelServo : rightJewelServo;
        leftJewelDetector = hardwareMap.colorSensor.get("leftCS");
        rightJewelDetector = leftJewelDetector;
        //rightJewelDetector = hardwareMap.colorSensor.get("rightCS");
        jewelDetector = alliance == PulsarAuto.Alliance.BLUE ? leftJewelDetector : rightJewelDetector;

        conveyor = hardwareMap.dcMotor.get("conveyor");
        conveyorLiftLeft = hardwareMap.servo.get("conveyorLiftLeft");
        conveyorLiftRight = hardwareMap.servo.get("conveyorLiftRight");

        intakeServoLeft = hardwareMap.servo.get("collectorServoLeft");
        intakeServoRight = hardwareMap.servo.get("collectorServoRight");
        leftIntake = hardwareMap.dcMotor.get("collectorLeft");
        rightIntake = hardwareMap.dcMotor.get("collectorRight");
    }

    private void initalizeIMU(BNO055IMU imu) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "IMUConfig.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu.initialize(parameters);
    }

    public void initializePositions(Telemetry tm) {
        intakeServoRight.setDirection(Servo.Direction.REVERSE);
        leftJewelServo.setDirection(Servo.Direction.REVERSE);
        rightJewelServo.setDirection(Servo.Direction.REVERSE);

        /*
            Previous Values (Maybe Better?):
            intakeServoLeft.setPosition(0.8);
            intakeServoRight.setPosition(0.85);
        */

        intakeUp();
        conveyorUp();
        jewelsUp();

        tm.addLine("Servo Positions: initialized");
        tm.update();
    }

    public void initializePositionsTeleop() {
        intakeServoRight.setDirection(Servo.Direction.REVERSE);
        leftJewelServo.setDirection(Servo.Direction.REVERSE);
        rightJewelServo.setDirection(Servo.Direction.REVERSE);

        jewelsUp();
    }

    public void jewelsUp() {
        leftJewelServo.setPosition(LEFT_JEWEL_UP_POSITON);
        rightJewelServo.setPosition(RIGHT_JEWEL_UP_POSITON);
    }

    public void conveyorUp() {
        conveyorLiftLeft.setPosition(CONVEYOR_LIFT_UP_POSITION);
        conveyorLiftRight.setPosition(CONVEYOR_LIFT_UP_POSITION);
    }

    public void conveyorDown() {
        conveyorLiftLeft.setPosition(CONVEYOR_LIFT_DOWN_POSITON);
        conveyorLiftRight.setPosition(CONVEYOR_LIFT_DOWN_POSITON);
    }

    public void intakeUp() {
        intakeServoLeft.setPosition(INTAKE_UP_POSITION);
        intakeServoRight.setPosition(INTAKE_UP_POSITION);
    }

    public void intakeDown() {
        intakeServoLeft.setPosition(INTAKE_DOWN_POSITION);
        intakeServoRight.setPosition(INTAKE_DOWN_POSITION);
    }
}
