package org.firstinspires.ftc.teamcode.lib;

import android.content.Context;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.competition.auto.PulsarAuto;
import org.redshiftrobotics.lib.RobotHardware;
import org.redshiftrobotics.lib.debug.DebugHelper;
import org.redshiftrobotics.lib.pid.PIDCalculator;

public class PulsarRobotHardware implements RobotHardware {
    public PulsarAuto.Alliance alliance;

    public final LinearOpMode opMode;
    public final Context appContext;

    private final HardwareMap hardwareMap;

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

    public final DcMotor conveyorMotor;
    public final Servo leftFlipperServo;
    public final Servo rightFlipperServo;

    public final Servo leftCollectionServo;
    public final Servo rightCollectionServo;
    public final DcMotor leftCollectionMotor;
    public final DcMotor rightCollectionMotor;

    // Constants
    public final double LEFT_JEWEL_UP_POSITON = 0.6;
    public final double LEFT_JEWEL_DOWN_POSITON = 0;
    public final double RIGHT_JEWEL_UP_POSITON = 0.5;
    public final double RIGHT_JEWEL_DOWN_POSITON = 0.8;

    public final double CONVEYOR_SPEED = 0.65;
    public final double FLIPPER_POSITION_SCALAR = 0.75;

    public final double COLLECTION_UP_POSITION = 1;
    public final double COLLECTION_DOWN_POSITION = 0;


    public PulsarRobotHardware(LinearOpMode opMode, PulsarAuto.Alliance alliance) {
        this.alliance = alliance;
        this.opMode = opMode;

        DebugHelper.setTelemetry(opMode.telemetry);

        hardwareMap = opMode.hardwareMap;
        appContext = opMode.hardwareMap.appContext;

        frontLeft = hardwareMap.dcMotor.get("r1m0");
        frontRight = hardwareMap.dcMotor.get("r1m1");
        backLeft = hardwareMap.dcMotor.get("r1m2");
        backRight = hardwareMap.dcMotor.get("r1m3");

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initalizeIMU(imu);

        leftJewelServo = hardwareMap.servo.get("r1s3");
        rightJewelServo = hardwareMap.servo.get("r1s2");
        jewelServo = alliance == PulsarAuto.Alliance.BLUE ? leftJewelServo : rightJewelServo;
        //jewelServo = leftJewelServo;

        leftJewelDetector = hardwareMap.colorSensor.get("r1c1");
        rightJewelDetector = hardwareMap.colorSensor.get("r1c2");
        //rightJewelDetector = leftJewelDetector;
        //jewelDetector = leftJewelDetector;
        jewelDetector = alliance == PulsarAuto.Alliance.BLUE ? leftJewelDetector : rightJewelDetector;

        conveyorMotor = hardwareMap.dcMotor.get("r2m1");
        leftFlipperServo = hardwareMap.servo.get("r2s1");
        rightFlipperServo = hardwareMap.servo.get("r2s2");
        leftFlipperServo.setDirection(Servo.Direction.REVERSE);

        leftCollectionServo = hardwareMap.servo.get("r1s1");
        rightCollectionServo = hardwareMap.servo.get("r1s0");
        leftCollectionMotor = hardwareMap.dcMotor.get("r2m2");
        rightCollectionMotor = hardwareMap.dcMotor.get("r2m3");
        leftCollectionServo.setDirection(Servo.Direction.REVERSE);
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
        rightCollectionServo.setDirection(Servo.Direction.REVERSE);
        leftJewelServo.setDirection(Servo.Direction.REVERSE);
        rightJewelServo.setDirection(Servo.Direction.REVERSE);

        /*
            Previous Values (Maybe Better?):
            leftCollectionServo.setPosition(0.8);
            rightCollectionServo.setPosition(0.85);
        */

        intakeUp();
        jewelsUp();

        tm.addLine("Servo Positions: initialized");
        tm.update();
    }

    public void initializePositionsTeleop() {
        rightCollectionServo.setDirection(Servo.Direction.REVERSE);
        leftJewelServo.setDirection(Servo.Direction.REVERSE);
        rightJewelServo.setDirection(Servo.Direction.REVERSE);

        jewelsUp();
    }

    public void jewelsUp() {
        leftJewelServo.setPosition(LEFT_JEWEL_UP_POSITON);
        rightJewelServo.setPosition(RIGHT_JEWEL_UP_POSITON);
    }

    public void intakeUp() {
        leftCollectionServo.setPosition(COLLECTION_UP_POSITION);
        rightCollectionServo.setPosition(COLLECTION_UP_POSITION);
    }

    public void intakeDown() {
        leftCollectionServo.setPosition(COLLECTION_DOWN_POSITION);
        rightCollectionServo.setPosition(COLLECTION_DOWN_POSITION);
    }

    public void intakeHalf() {
        double pos = (COLLECTION_DOWN_POSITION + COLLECTION_UP_POSITION) / 2;
        leftCollectionServo.setPosition(pos);
        rightCollectionServo.setPosition(pos);
    }

    /*
     * When this helper method is used in a turning function, correctionAngular
     * should be added to a power constant in order to make the wheels turn at a reasonable
     * speed. For small corrections during straight movement though, this is unnecessary.
     */
    @Override
    public void applyMotorPower(double velocityX, double velocityY,  double correctionAngular) {
        /*
         * Values are added and subtracted here based on the direction the wheels need to go on a
         * mecanum chassis to perform specific movements. For instance, Y velocity is always added,
         * because all motors go the same direction when moving forwards and backwards. Angular
         * movement and strafing require changes, because different wheels must move in different
         * directions to make the movement possible.
         */

        // FIXME: dividing all corrections by 2000 to prevent overflow is bad
        double frontLeftPower = velocityY  - velocityX  + correctionAngular/2000.0;
        double frontRightPower = velocityY + velocityX  - correctionAngular/2000.0;
        double backRightPower = velocityY - velocityX - correctionAngular/2000.0;
        double backLeftPower = velocityY + velocityX + correctionAngular/2000.0;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    @Override
    public PIDCalculator.PIDTuning getTurningTuning() {
        return new PIDCalculator.PIDTuning(100, 0.0069, 0);
    }

    @Override
    public PIDCalculator.PIDTuning getStraightTurning() {
        return new PIDCalculator.PIDTuning(100, /* 2.0e-4 */ 0, 0);
    }

    @Override
    public double getTurningAngleThreshold() {
        return 1;
    }

    @Override
    public Context getAppContext() {
        return appContext;
    }

    @Override
    public LinearOpMode getOpMode() {
        return opMode;
    }

    @Override
    public BNO055IMU getIMU() {
        return imu;
    }
}
