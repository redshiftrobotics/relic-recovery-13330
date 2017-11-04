package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Duncan on 9/19/2017.
 */

@TeleOp(name = "Pulsar Teleop")
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

    Servo collectorLeft;
    Servo collectorRight;



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

        collectorLeft = hardwareMap.servo.get("collectorLeft");
        collectorRight = hardwareMap.servo.get("collectorRight");

        setServos();
    }

    /*
    Motor layout:



    fl fr

    bl br
     */

    float frontLeftPower, frontRightPower, backLeftPower, backRightPower;

    static float CONVEYOR_BELT_POWER_SCALAR = 0.5f;

    @Override
    public void loop() {
        controlConveyor();
        drive();
    }

    private void controlConveyor() {

    //    int gpadA = gamepad1.a ? 1 : 0;
     //   int gpadB = gamepad1.b ? 1 : 0;
     //   float gamepad1Power = (float) (gpadA + (gpadB & (~gpadA + 2) * -1));

        conveyor.setPower(gamepad1.left_bumper ? -0.5f : (gamepad1.left_trigger * CONVEYOR_BELT_POWER_SCALAR));
        //conveyorLift.setPosition(gamepad1.x ? 1.0f : (gamepad1.y ? 0.0f : conveyorLift.getPosition()));
        leftIntake.setPower(gamepad1.right_bumper ? -0.5f : (gamepad1.right_trigger/2));
        rightIntake.setPower(gamepad1.right_bumper ? -0.5f : (gamepad1.right_trigger/2));
    }

    private void setServos() {
        leftJewel.setPosition(0.2);
        rightJewel.setPosition(0.55);

        collectorLeft.setPosition(0.38);
        collectorRight.setPosition(0.66);
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