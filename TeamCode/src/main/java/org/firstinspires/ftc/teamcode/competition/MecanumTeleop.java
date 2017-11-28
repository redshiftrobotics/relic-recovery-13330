package org.firstinspires.ftc.teamcode.competition;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.redshiftrobotics.lib.config.ConfigurationManager;

/**
 * Created by Duncan on 9/19/2017.
 */

@TeleOp(name = "Pulsar Teleop")
public class MecanumTeleop extends OpMode{
    private static final double TICKS_PER_CM = 1/ ConfigurationManager.getSharedInstance().getDouble("encoderTicksPerCM");

    private static final ConfigurationManager hwPos = ConfigurationManager.getSharedInstance().getConfig("hw:pos");


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
        telemetry.addLine("Initializing for TeleOp!");
        telemetry.update();

        try {
            ConfigurationManager.setup("default");
        } catch (Exception e) {
            telemetry.addLine("ERROR!!!!! EXITING");
            telemetry.update();
            Log.e("PulsarAuto", "configuration manager error", e);
            return;
        }

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

        telemetry.addLine("Ready for TeleOp!");
        telemetry.update();
    }

    /*
    Motor layout:



    fl fr

    bl br
     */

    float frontLeftPower, frontRightPower, backLeftPower, backRightPower;

    static float CONVEYOR_BELT_POWER_SCALAR = (float) ConfigurationManager.getSharedInstance().getDouble("conveyorPowerScalar");

    @Override
    public void loop() {
        controlConveyor();
        drive();
        telemetry.addData("fl", frontLeft.getCurrentPosition());
        telemetry.addData("bl", backLeft.getCurrentPosition());
        telemetry.update();
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
        leftJewel.setPosition(ConfigurationManager.getSharedInstance().getDouble("jewel:left:up"));
        rightJewel.setPosition(ConfigurationManager.getSharedInstance().getDouble("jewel:right:up"));

        collectorLeft.setPosition(ConfigurationManager.getSharedInstance().getDouble("collector:left:down"));
        collectorRight.setPosition(ConfigurationManager.getSharedInstance().getDouble("collector:right:down"));

        conveyorLift.setPosition(ConfigurationManager.getSharedInstance().getDouble("conveyor:teleopPosition"));
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