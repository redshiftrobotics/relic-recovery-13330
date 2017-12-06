package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;

/**
 * Created by Duncan on 11/15/2017.
 */
@TeleOp(name="Pulsar Teleop", group="Pulsar")
public class PulsarTeleop extends LinearOpMode{
    PulsarRobotHardware hw;

    static final double CONVEYOR_BELT_POWER_SCALAR = 0.85;
    static final double DEFAULT_CONVEYOR_SPEED = 0.5;

    static final double INTAKE_POWER_SCALAR = 0.5;
    static final double DEFAULT_INTAKE_SPEED = -0.5;

    static final double INTAKE_UP_POSITION = 0.60;
    static final double INTAKE_DOWN_POSITION = 0;

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
    public void runOpMode() throws InterruptedException {
        init_();
        waitForStart();
        while (opModeIsActive()) { loop_(); }
    }

    public void init_() {
        telemetry.addLine("Initializing for TeleOp!");
        telemetry.update();

        hw = new PulsarRobotHardware(hardwareMap, null);

        setServos();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "IMUConfig.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        hw.imu.initialize(parameters);

        telemetry.addLine("Ready for TeleOp!");
        telemetry.update();
    }

    private void setServos() {
      //  hw.leftJewelServo.setPosition(0.2);
      //  hw.rightJewelServo.setPosition(0.55);
        hw.initializePositionsTeleop();

        hw.intakeServoRight.setDirection(Servo.Direction.REVERSE);
/*
        hw.intakeServoLeft.setPosition(0.38);
        hw.intakeServoRight.setPosition(0.66);

*/
        hw.conveyorLift.setPosition(0.28);
    }

    public void loop_() {
        UpdateMovements(gamepad1);
        //UpdatePConstants(gamepad2);
        CalculateMovements();
        ControlRobot();
        ControlConveyor(gamepad2);
        StoreRotation(gamepad2);
        UpdateTelemetry();
        idle();
    }

    public void StoreRotation(Gamepad pad) { //Function that updates the stored rotation, can be customized to use a specific controller
        if (pad.b) cryptoboxAngle = currentAngle;
        if (pad.x) targetAngle = cryptoboxAngle;
    }

    public void UpdateMovements(Gamepad pad){ //Function that updates the movement constants, can be customized to use a specific controller
        xPower = pad.right_stick_x;
        yPower = -pad.right_stick_y;
        anglePower = -pad.left_stick_x;
    }

    public void UpdatePConstants(Gamepad pad){ //Function that updates the P constants, can be customized to use a specific controller
        if (pad.left_bumper != lastLB) { pConstant += 0.2; }
        if (pad.right_bumper != lastRB) { pConstant -= 0.2; }
        lastLB = pad.left_bumper; lastRB = pad.right_bumper;
    }

    private void ControlConveyor(Gamepad pad) {
        hw.leftIntake.setDirection(pad.left_bumper ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        hw.rightIntake.setDirection(pad.right_bumper ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        double scalar = INTAKE_POWER_SCALAR;
        if (pad.right_bumper) { scalar *= 0.5; }
        if (gamepad1.back) { scalar = 1; }
        hw.leftIntake.setPower(pad.left_trigger * (scalar));
        hw.rightIntake.setPower(pad.right_trigger * (scalar));

        // ----
        if (gamepad1.back) {
            hw.conveyor.setPower(pad.left_stick_y);
        } else {
            hw.conveyor.setPower(pad.left_stick_y * CONVEYOR_BELT_POWER_SCALAR);
        }


        // Up/Down
        if (pad.a) {
            hw.intakeServoLeft.setPosition(INTAKE_DOWN_POSITION);
            hw.intakeServoRight.setPosition(INTAKE_DOWN_POSITION);
        }
        if (pad.y) {
            hw.intakeServoLeft.setPosition(INTAKE_UP_POSITION);
            hw.intakeServoRight.setPosition(INTAKE_UP_POSITION);
        }

        if (pad.dpad_up) {
            hw.conveyorLift.setPosition(0.45);
        } else if (pad.dpad_down) {
            hw.conveyorLift.setPosition(0.28);
        }
    }

    public void ControlRobot(){ //Function to control all hardware devices on the robot
        hw.frontLeft.setPower(flPower);
        hw.frontRight.setPower(frPower);
        hw.backLeft.setPower(blPower);
        hw.backRight.setPower(brPower);
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
        currentAngle = hw.imu.getAngularOrientation().firstAngle + 180f;
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
