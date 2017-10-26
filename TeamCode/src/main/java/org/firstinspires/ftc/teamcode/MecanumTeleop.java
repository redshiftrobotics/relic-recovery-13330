package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Duncan on 9/19/2017.
 */

@TeleOp(name = "mecanum")
public class MecanumTeleop extends OpMode{

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



    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyor = hardwareMap.dcMotor.get("conveyor");
        conveyorLift = hardwareMap.servo.get("conveyorLift");
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");
        leftJewel = hardwareMap.servo.get("leftJewel");
        rightJewel = hardwareMap.servo.get("rightJewel");
    }

    /*
    Motor layout:



    fl fr

    bl br
     */

    float frontLeftPower, frontRightPower, backLeftPower, backRightPower;

    @Override
    public void loop() {
        controlConveyor();
        controlJewel();
        drive();

    }

    private void controlConveyor(){
        conveyor.setPower(gamepad1.a?1.0f:(gamepad1.b?-1.0f:0.0));
        conveyorLift.setPosition(gamepad1.x?1.0f:(gamepad1.y?0.0f:conveyorLift.getPosition()));
        leftIntake.setPower(gamepad1.right_bumper?-1.0f:(gamepad1.right_trigger));
        rightIntake.setPower(gamepad1.right_bumper?-1.0f:(gamepad1.right_trigger));
    }

    private void controlJewel(){
        leftJewel.setPosition(0.0);
        rightJewel.setPosition(0.0);
    }

    private void drive() {
        frontLeftPower = (gamepad1.right_stick_y /*FORWARD POWER*/) - (gamepad1.right_stick_x /*SIDE POWER*/) - (gamepad1.left_stick_x /*TURN POWER*/);
        frontRightPower = (gamepad1.right_stick_y /*FORWARD POWER*/) + (gamepad1.right_stick_x /*SIDE POWER*/) + (gamepad1.left_stick_x /*TURN POWER*/);
        backRightPower = (gamepad1.right_stick_y /*FORWARD POWER*/) - (gamepad1.right_stick_x /*SIDE POWER*/) + (gamepad1.left_stick_x /*TURN POWER*/);
        backLeftPower = (gamepad1.right_stick_y /*FORWARD POWER*/) + (gamepad1.right_stick_x /*SIDE POWER*/) - (gamepad1.left_stick_x /*TURN POWER*/);



        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
    }
}