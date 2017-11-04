package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ariporad on 2017-11-04.
 */

public class SizingCubeFitter {
    OpMode ctx;
    Servo conveyorLift;
    Servo leftJewel;
    Servo rightJewel;

    public SizingCubeFitter(OpMode context) {
        ctx = context;
        conveyorLift = ctx.hardwareMap.servo.get("conveyorLift");
        leftJewel = ctx.hardwareMap.servo.get("leftJewel");
        rightJewel = ctx.hardwareMap.servo.get("rightJewel");
    }

    public void fitInSizingCube() {
        // Interrupted exception only can occur if we sleep, which we don't, so it's fine to ignore.
        try { fitInSizingCube(false); } catch (InterruptedException e) {};
    }

    public void fitInSizingCube(boolean block) throws InterruptedException {
        conveyorLift.setPosition(0.43);
        leftJewel.setPosition(0.05);
        rightJewel.setPosition(0.60);

        if (block) Thread.sleep(10000);
    }
}
