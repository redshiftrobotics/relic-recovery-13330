package org.firstinspires.ftc.teamcode.competition.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;
import org.redshiftrobotics.lib.pid.imu.IMU;
import org.redshiftrobotics.lib.pid.imu.IMUWrapper;

/**
 * Created by Duncan on 1/20/2018.
 */

@TeleOp(name="Pulsar State Teleop", group="Pulsar")
public class PulsarStateTeleop extends LinearOpMode {

    private static final boolean USE_PID = true;

    private PulsarRobotHardware hw;

    private IMU imu;

    private static final double DRIVE_POWER_SCALAR = 0;
    private static final double COLLECTOR_POWER_SCALAR = 0.5;
    private static final double CONVEYOR_POWER = 0.5f;

    private double xDrivePower = 0;
    private double yDrivePower = 0;
    private double xCollectionPower = 0;
    private double yCollectionPower = 0;

    private double pPower = 0;
    private double anglePower = 0;
    private double targetAngle = 180;
    private double currentAngle = 180;
    private double lastAngle = 180;
    private double pConstant = 5;

    private boolean isTurning = false;
    private boolean flipperUp = false;
    private boolean conveyorOn = true;

    //Drive motor power
    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;
    private double maxPower = 0;

    private double relicPower = 0;
    private double collectorLeft = 0;
    private double collectorRight = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing for TeleOp!");
        telemetry.update();

        hw = new PulsarRobotHardware(this, null);
        imu = new IMUWrapper(hw.imu);

        hw.initializePositionsTeleop();
        hw.conveyorDown();

        telemetry.addLine("Ready for TeleOp!");
        telemetry.addLine();
        telemetry.addLine("Press START to Begin!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            ReadDriverControls(gamepad1);
            ReadOperatorControls(gamepad2);
            ComputePLoop();
            CalculateCollection();
            NormalizeMotorPowers();
            //Update all hardware based on user input, these must be called last
            UpdateMotors();
            UpdateServos();
        }
    }

    private void UpdateMotors(){
        //Update drive motors
        hw.frontLeft.setPower(flPower);
        hw.frontRight.setPower(frPower);
        hw.backLeft.setPower(blPower);
        hw.backRight.setPower(brPower);

        //Update collector motors
        hw.leftCollectionMotor.setPower(collectorLeft);
        hw.rightCollectionMotor.setPower(collectorRight);
        if(conveyorOn) hw.conveyorMotor.setPower(CONVEYOR_POWER);
        else hw.conveyorMotor.setPower(-CONVEYOR_POWER);

        //Update relic motors
        //hw.relic.setPower(relicPower);
    }

    private void UpdateServos(){
        if(flipperUp){
            hw.leftFlipperServo.setPosition(hw.FLIPPER_UP_POSITION);
            hw.leftFlipperServo.setPosition(hw.FLIPPER_UP_POSITION);
        }else{
            hw.leftFlipperServo.setPosition(hw.FLIPPER_DOWN_POSITION);
            hw.leftFlipperServo.setPosition(hw.FLIPPER_DOWN_POSITION);
        }
    }

    private void ReadDriverControls(Gamepad driver){
        xDrivePower = driver.right_stick_x * DRIVE_POWER_SCALAR;
        yDrivePower = -driver.left_stick_y * DRIVE_POWER_SCALAR;
        anglePower = driver.left_stick_x * DRIVE_POWER_SCALAR;
    }

    private void ReadOperatorControls(Gamepad operator){
        xCollectionPower = operator.right_stick_x * COLLECTOR_POWER_SCALAR;
        yCollectionPower = -operator.right_stick_y * COLLECTOR_POWER_SCALAR;
        flipperUp = operator.a;
        conveyorOn = !operator.b;
    }

    private void CalculateCollection(){
        collectorLeft = yCollectionPower - xCollectionPower;
        collectorRight = yCollectionPower + xCollectionPower;
    }

    private void ComputePLoop() { //Function to control the robot's movements, this includes calculating P and calculating the motor powers
        lastAngle = currentAngle;
        currentAngle = imu.getAngularRotationX();

        if (anglePower != 0) isTurning = true;
        if (Math.abs(lastAngle - currentAngle) < 1 && isTurning) isTurning = false;

        if (isTurning) {
            targetAngle = currentAngle;
            pPower = 0.0;
        } else {
            if (currentAngle + 360.0 - targetAngle <= 180.0) {
                pPower = (currentAngle - targetAngle + 360.0) * pConstant;
            } else if (targetAngle + 360.0 - currentAngle <= 180.0) {
                pPower = (targetAngle - currentAngle + 360.0) * -1.0 * pConstant;
            } else if (currentAngle - targetAngle <= 180.0) {
                pPower = (currentAngle - targetAngle) * pConstant;
            }
        }

        pPower = Range.clip(pPower / -180.0, -1.0, 1.0);

        flPower = yDrivePower - xDrivePower + pPower + anglePower;
        frPower = yDrivePower + xDrivePower - pPower - anglePower;
        blPower = yDrivePower + xDrivePower + pPower + anglePower;
        brPower = yDrivePower - xDrivePower - pPower - anglePower;
    }

    private void NormalizeMotorPowers() {
        //Calculate which motor has the highest non-normalized power
        maxPower = Math.abs(flPower);
        if (Math.abs(frPower) > maxPower) maxPower = Math.abs(frPower);
        else if (Math.abs(blPower) > maxPower) maxPower = Math.abs(blPower);
        else if (Math.abs(brPower) > maxPower) maxPower = Math.abs(brPower);

        //Scale all motor power to fit within -1.0 to 1.0 based on maxPower
        flPower = flPower / maxPower;
        frPower = frPower / maxPower;
        blPower = blPower / maxPower;
        brPower = brPower / maxPower;
    }
}
