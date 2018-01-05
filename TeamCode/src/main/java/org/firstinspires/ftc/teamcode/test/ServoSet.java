package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by adam on 11/1/17.
 */
@TeleOp(name="Servo Set 2")
@Disabled
public class ServoSet extends OpMode{

    Servo leftCollector;
    Servo rightCollector;
    DcMotor conveyor;
    DcMotor leftIntake;
    DcMotor rightIntake;

    boolean aBounce = false;
    boolean bBounce = false;
    boolean xBounce = false;
    boolean yBounce = false;

    double leftPos = 0.5;
    double rightPos = 0.5;
    @Override
    public void init() {
        leftCollector = hardwareMap.servo.get("collectorLeft");
        rightCollector = hardwareMap.servo.get("collectorRight");
        conveyor = hardwareMap.dcMotor.get("conveyor");
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");
    }

    @Override
    public void loop() {

        if(gamepad1.a&&!aBounce){
            aBounce=true;
            leftPos+=0.01;
        }else if(aBounce&&!gamepad1.a){
            aBounce=false;
        }
        if(gamepad1.b&&!bBounce){
            bBounce=true;
            leftPos-=0.01;
        }else if(bBounce&&!gamepad1.b){
            bBounce=false;
        }
        if(gamepad1.x&&!xBounce){
            xBounce=true;
            rightPos+=0.01;
        }else if(xBounce&&!gamepad1.x){
            xBounce=false;
        }
        if(gamepad1.y&&!yBounce){
            yBounce=true;
            rightPos-=0.01;
        }else if(yBounce&&!gamepad1.y){
            yBounce=false;
        }


        leftCollector.setPosition(leftPos);
        rightCollector.setPosition(rightPos);
        conveyor.setPower(gamepad1.left_bumper ? -0.5f : (gamepad1.left_trigger/2));
        leftIntake.setPower(gamepad1.right_bumper ? -0.5f : (gamepad1.right_trigger/2));
        rightIntake.setPower(gamepad1.right_bumper ? -0.5f : (gamepad1.right_trigger/2));
        telemetry.addData("Left", leftCollector.getPosition());
        telemetry.addData("Right", rightCollector.getPosition());
        telemetry.update();
    }
}
