package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Duncan on 11/15/2017.
 */
@TeleOp(name="Pulsar P Teleop", group="Pulsar")
public class PulsarPTeleop extends OpMode{

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    DcMotor conveyor;
    Servo conveyorLift;

    DcMotor leftIntake;
    DcMotor rightIntake;

    Servo leftJewel;
    Servo rightJewel;

    Servo collectorLeft;
    Servo collectorRight;

    BNO055IMU imu;

    double xPower = 0;
    double yPower = 0;
    double pPower = 0;
    double anglePower = 0;
    double targetAngle = 180;
    double currentAngle = 180;
    double pConstant = 3;

    double flPower = 0;
    double frPower = 0;
    double blPower = 0;
    double brPower = 0;

    long currentTime;
    long lastTime;

    String fuckingDebug = "nothing";

    boolean lastRB = false;
    boolean lastLB = false;

    @Override
    public void init() {
        telemetry.addLine("Initializing for TeleOp!");
        telemetry.update();

        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //conveyor = hardwareMap.dcMotor.get("conveyor");
        //conveyorLift = hardwareMap.servo.get("conveyorLift");
        //leftIntake = hardwareMap.dcMotor.get("leftIntake");
        //rightIntake = hardwareMap.dcMotor.get("rightIntake");
        //leftJewel = hardwareMap.servo.get("leftJewel");
        //rightJewel = hardwareMap.servo.get("rightJewel");

        //collectorLeft = hardwareMap.servo.get("collectorLeft");
        //collectorRight = hardwareMap.servo.get("collectorRight");

        //setServos();

        lastTime = System.currentTimeMillis();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "IMUConfig.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        currentTime = System.currentTimeMillis();

        telemetry.addLine("Ready for TeleOp!");
        telemetry.update();
    }

    private void setServos() {
        leftJewel.setPosition(0.2);
        rightJewel.setPosition(0.55);

        collectorLeft.setPosition(0.38);
        collectorRight.setPosition(0.66);

        conveyorLift.setPosition(0.28);
    }

    @Override
    public void loop() {
        UpdateTime();
        UpdateController(gamepad1);
        CalculateP();
        Drive();
        UpdateTelemetry();
    }

    public void UpdateTime(){
        lastTime = currentTime;
        currentTime = System.currentTimeMillis();
    }

    public void UpdateController(Gamepad pad){
        xPower = pad.right_stick_x;
        yPower = pad.right_stick_y;
        anglePower = pad.left_stick_x;

        if (pad.left_bumper != lastLB) { pConstant += 0.2; }
        if (pad.right_bumper != lastRB) { pConstant -= 0.2; }
        lastLB = pad.left_bumper; lastRB = pad.right_bumper;
    }

    public void Drive(){
        flPower = yPower - xPower + pPower + anglePower;
        frPower = yPower + xPower - pPower - anglePower;
        blPower = yPower + xPower + pPower + anglePower;
        brPower = yPower - xPower - pPower - anglePower;
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    public void UpdateTelemetry(){
        telemetry.addData("Current Angle", currentAngle + ", Target Angle: " + targetAngle);
        telemetry.addData("debug", fuckingDebug);
        telemetry.addData("pConstant", pConstant);
        telemetry.addData("FL", flPower);
        telemetry.addData("FR", frPower);
        telemetry.addData("BL", blPower);
        telemetry.addData("BR", brPower);
        telemetry.addData("XP", xPower);
        telemetry.addData("YP", yPower);
        telemetry.addData("PP", pPower);
        telemetry.addData("AP", anglePower);
        telemetry.update();
    }

    public void CalculateP(){
        currentAngle = imu.getAngularOrientation().firstAngle + 180f;
        if(anglePower!=0){
            targetAngle=currentAngle;
            pPower = 0f;
        }else {
            if (currentAngle + 360f - targetAngle <= 180f) {
                pPower = (currentAngle - targetAngle + 360f) * pConstant;
                fuckingDebug = "first";
            } else if (targetAngle + 360f - currentAngle <= 180f) {
                pPower = (targetAngle - currentAngle + 360f) * -1f * pConstant;
                fuckingDebug = "second";
            } else if (currentAngle - targetAngle <= 180f) {
                pPower = (currentAngle - targetAngle) * pConstant;
                fuckingDebug = "third";
            } else {
                pPower = 7;
                fuckingDebug = "pizza";
            }
        }
        pPower = Range.clip(pPower / -180f, -1, 1);

    }
}
