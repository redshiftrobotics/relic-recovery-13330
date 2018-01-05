package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Duncan on 9/19/2017.
 */

public class ColorSensorJewelDetectorTest extends LinearOpMode{

    private String team = "red"; //Just for testing will be set up by a custom setting later

    ColorSensor cs;
    Servo actuator;

    @Override
    public void runOpMode() throws InterruptedException {
        cs = hardwareMap.colorSensor.get("cs");
        actuator = hardwareMap.servo.get("actuator");
        actuator.setPosition(0.5); //set actuator to upright position
        waitForStart();
        actuator.setPosition(0.0); //deploy actuator
        if(cs.red()>cs.blue()){
            if(team=="red"){
                telemetry.addData("Move", "Backward");
            }else if(team=="blue"){
                telemetry.addData("Move", "Forward");
            }
        }else if(cs.blue()>cs.red()){
            if(team=="red"){
                telemetry.addData("Move", "Forward");
            }else if(team=="blue"){
                telemetry.addData("Move", "Backward");
            }
        }
        telemetry.update();
    }
}
