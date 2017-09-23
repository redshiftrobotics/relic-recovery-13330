package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Duncan on 9/23/2017.
 */

@TeleOp(name="2856 Drive 2016-2017", group="Outreach")
public class Drive2856Old extends OpMode {

    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor[] capDrive = new DcMotor[2];
    DcMotor flicker;
    DcMotor collector;

    Servo latch;
    Servo mantisLeft;
    Servo mantisRight;
    Servo aim;

    @Override
    public void init() {
        leftDrive = hardwareMap.dcMotor.get("LDrive");
        rightDrive = hardwareMap.dcMotor.get("RDrive");
        capDrive[0] = hardwareMap.dcMotor.get("cDrive0");
        capDrive[1] = hardwareMap.dcMotor.get("cDrive1");
        flicker = hardwareMap.dcMotor.get("flicker");
        flicker.setDirection(DcMotorSimple.Direction.REVERSE);
        collector = hardwareMap.dcMotor.get("collector");
        collector.setDirection(DcMotorSimple.Direction.REVERSE);

        latch = hardwareMap.servo.get("latch");
        mantisLeft = hardwareMap.servo.get("mantisLeft");
        mantisRight = hardwareMap.servo.get("mantisRight");
        aim = hardwareMap.servo.get("aim");

        aim.setPosition(0.59f);
        mantisLeft.setPosition(0.95);
        mantisRight.setPosition(0.25);
        latch.setPosition(0.15);
    }

    @Override
    public void loop() {
        Shoot();
        Cap();
        Drive();
    }

    void Shoot(){
        if(gamepad1.right_trigger>0.1){
            flicker.setPower(1.0);
        }else if(gamepad1.right_bumper){
            flicker.setPower(-0.5);
        }else{
            flicker.setPower(0.0);
        }
        if(gamepad1.left_trigger>0.1){
            collector.setPower(1.0);
        }else if(gamepad1.left_bumper){
            collector.setPower(-1.5);
        }else{
            collector.setPower(0.0);
        }

        if(gamepad1.dpad_up){
            aim.setPosition(0.2f);
        }else if(gamepad1.dpad_down){
            aim.setPosition(0.51f);
        }else if(gamepad1.dpad_left){
            aim.setPosition(0.54f);
        }else if(gamepad1.dpad_right){
            aim.setPosition(0.59f);
        }
    }

    void Cap(){
        capDrive[0].setPower(Range.clip(gamepad1.left_stick_y,-1.0,1.0));
        capDrive[1].setPower(Range.clip(gamepad1.left_stick_y,-1.0,1.0));

        if(gamepad1.y) {
            mantisLeft.setPosition(0.2);
            mantisRight.setPosition(.65);
            latch.setPosition(1);
        } else if (gamepad1.x) {
            mantisLeft.setPosition(0.45);
            mantisRight.setPosition(0.4);
        }else if(gamepad1.a){
            mantisLeft.setPosition(1);
            mantisRight.setPosition(0.2);
        }
    }

    void Drive(){
        leftDrive.setPower(Range.clip(-gamepad1.right_stick_y+gamepad1.right_stick_x,-1.0,1.0));
        rightDrive.setPower(Range.clip(gamepad1.right_stick_y+gamepad1.right_stick_x,-1.0,1.0));
    }
}
