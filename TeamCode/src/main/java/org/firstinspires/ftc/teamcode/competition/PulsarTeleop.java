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
@TeleOp(name="Pulsar Teleop", group="Pulsar")
public class PulsarTeleop extends OpMode{

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
    double cryptoboxAngle = 104;
    double lastAngle = 180;
    double pConstant = 3;

    boolean isTurning = false;

    double flPower = 0;
    double frPower = 0;
    double blPower = 0;
    double brPower = 0;
    double maxPower = 0;

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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "IMUConfig.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

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
        UpdateMovements(gamepad1);
        UpdatePConstants(gamepad2);
        CalculateMovements();
        ControlRobot();
        StoreRotation(gamepad2);
        UpdateTelemetry();
    }

    public void StoreRotation(Gamepad pad) { //Function that updates the stored rotation, can be customized to use a specific controller
        if (pad.back) cryptoboxAngle = currentAngle;
        if (pad.a) targetAngle = cryptoboxAngle;
    }

    public void UpdateMovements(Gamepad pad){ //Function that updates the movement constants, can be customized to use a specific controller
        xPower = pad.right_stick_x;
        yPower = pad.right_stick_y;
        anglePower = pad.left_stick_x;
    }

    public void UpdatePConstants(Gamepad pad){ //Function that updates the P constants, can be customized to use a specific controller
        if (pad.left_bumper != lastLB) { pConstant += 0.2; }
        if (pad.right_bumper != lastRB) { pConstant -= 0.2; }
        lastLB = pad.left_bumper; lastRB = pad.right_bumper;
    }

    public void ControlRobot(){ //Function to control all hardware devices on the robot
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    public void UpdateTelemetry(){ //Function to telemetry out important values
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

    public void CalculateMovements(){ //Function to control the robot's movements, this includes calculating P and calculating the motor powers
        lastAngle = currentAngle;
        currentAngle = imu.getAngularOrientation().firstAngle + 180f;
        if (anglePower != 0) isTurning = true;
        if (Math.abs(lastAngle - currentAngle) < 1 && isTurning) isTurning = false;
        if(isTurning){
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

        flPower = (yPower - xPower + pPower + anglePower);
        frPower = (yPower + xPower - pPower - anglePower);
        blPower = (yPower + xPower + pPower + anglePower);
        brPower = (yPower - xPower - pPower - anglePower);

        maxPower = Math.abs(flPower);
        if(Math.abs(frPower)>maxPower) maxPower=Math.abs(frPower);
        if(Math.abs(blPower)>maxPower) maxPower=Math.abs(blPower);
        if(Math.abs(brPower)>maxPower) maxPower=Math.abs(brPower);
        if(maxPower<1) maxPower=1;

        flPower /= maxPower;
        frPower /= maxPower;
        blPower /= maxPower;
        brPower /= maxPower;
    }
}
