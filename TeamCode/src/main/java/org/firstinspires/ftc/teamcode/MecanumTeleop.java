package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.redshiftrobotics.lib.AutoOpMode;
import org.redshiftrobotics.lib.Positioner;
import org.redshiftrobotics.lib.encoder.DcMotorEncoder;
import org.redshiftrobotics.lib.encoder.EncoderPositioner;

/**
 * Created by Duncan on 9/19/2017.
 */

@TeleOp(name = "mecanum")
public class MecanumTeleop extends OpMode {
    static double TICKS_PER_CM = 1/1440;

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    Positioner positioner;

    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        positioner = new EncoderPositioner(new DcMotorEncoder(frontLeft), new DcMotorEncoder(backLeft), TICKS_PER_CM);
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /*
    Motor layout:



    fl fr

    bl br
     */

    float frontLeftPower, frontRightPower, backLeftPower, backRightPower;

    @Override
    public void loop() {
        drive();
        telemetry.addData("pos", positioner.getPosition());
        telemetry.addData("fl", frontLeft.getCurrentPosition());
        telemetry.addData("bl", backLeft.getCurrentPosition());
        telemetry.update();
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