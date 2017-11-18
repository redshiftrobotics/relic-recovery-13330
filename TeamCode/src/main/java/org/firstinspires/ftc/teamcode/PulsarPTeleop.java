package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

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
    double anglePower = 0;
    double targetAngle = 0;
    double currentAngle = 0;
    double pConstant = 1;

    double flPower = 0;
    double frPower = 0;
    double blPower = 0;
    double brPower = 0;

    long currentTime;
    long lastTime;



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

        imu = hardwareMap.get(BNO055IMU.class, "imu");

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
        targetAngle += pad.left_stick_x * (currentTime - lastTime);
        targetAngle %= 360;
    }

    public void Drive(){
        flPower = yPower + xPower + anglePower;
        frPower = yPower + xPower - anglePower;
        blPower = yPower - xPower + anglePower;
        brPower = yPower - xPower - anglePower;
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    public void UpdateTelemetry(){
        telemetry.addData("Current Angle", currentAngle + ", Target Angle: " + targetAngle);
        telemetry.addData("FL", flPower);
        telemetry.addData("FR", frPower);
        telemetry.addData("BL", blPower);
        telemetry.addData("BR", brPower);
        telemetry.addData("XP", xPower);
        telemetry.addData("YP", yPower);
        telemetry.addData("AP", anglePower);
        telemetry.update();
    }

    public void CalculateP(){
        currentAngle = imu.getAngularOrientation().firstAngle;
        if (currentAngle + 360 - targetAngle <= 180) {
            anglePower = (currentAngle -  targetAngle + 360) * pConstant;
        } else if (targetAngle + 360 - currentTime <= 180) {
            anglePower = (targetAngle - currentTime + 360) * -1 * pConstant;
        } else if (currentTime -  targetAngle <= 180) {
            anglePower = (currentTime -  targetAngle) * pConstant;
        }
    }
}
