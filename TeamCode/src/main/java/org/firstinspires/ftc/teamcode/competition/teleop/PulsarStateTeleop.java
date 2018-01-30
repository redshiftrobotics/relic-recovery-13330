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

    private static final boolean USE_PID = false;
    private final double FLIPPER_MIN_POSITION = 0.01;

    private PulsarRobotHardware hw;

    private IMU imu;

    private static final double DRIVE_POWER_SCALAR = 1;
    private static final double COLLECTOR_POWER_LEFT_SCALAR = 1;
    private static final double COLLECTOR_POWER_RIGHT_SCALAR = 1;
    private static final double CONVEYOR_POWER = 0.8f;

    private double xDrivePower = 0;
    private double yDrivePower = 0;

    private double pPower = 0;
    private double anglePower = 0;
    private double targetAngle = 180;
    private double currentAngle = 180;
    private double lastAngle = 180;
    private double pConstant = 0;

    private boolean isTurning = false;
    private boolean conveyorForward = true;
    private boolean collectionUp = false;
    private boolean maxConveyorPower = false;

    private double flipperPosition = 0;
    private double movementScalar = 0;
    private double rotationScalar = 0;

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

        telemetry.addLine("Ready for TeleOp!");
        telemetry.addLine();
        telemetry.addLine("Press START to Begin!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            ReadDriverControls(gamepad1);
            ReadOperatorControls(gamepad2);
            ComputePLoop();
            NormalizeMotorPowers();
            //Update all hardware based on user input, these must be called last
            UpdateMotors();
            UpdateServos();
            telemetry.update();
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

        double rawConveyorSpeed = maxConveyorPower ? 1 : CONVEYOR_POWER;
        if(conveyorForward) hw.conveyorMotor.setPower(rawConveyorSpeed);
        else hw.conveyorMotor.setPower(-rawConveyorSpeed);

        //Update relic motors
        //hw.relic.setPower(relicPower);
    }

    private void UpdateServos(){
        double realFlipperPosition = hw.FLIPPER_POSITION_SCALAR * flipperPosition;
        if(realFlipperPosition< FLIPPER_MIN_POSITION) realFlipperPosition = FLIPPER_MIN_POSITION;
        hw.leftFlipperServo.setPosition(realFlipperPosition);
        hw.rightFlipperServo.setPosition(realFlipperPosition);

        if (!collectionUp) {
            hw.leftCollectionServo.setPosition(hw.COLLECTION_UP_POSITION);
            hw.rightCollectionServo.setPosition(hw.COLLECTION_UP_POSITION);
        } else {
            hw.leftCollectionServo.setPosition(hw.COLLECTION_DOWN_POSITION);
            hw.rightCollectionServo.setPosition(hw.COLLECTION_DOWN_POSITION);
        }
        telemetry.addData("collection", collectionUp);
        telemetry.addData("collservo L", hw.leftCollectionServo.getPosition());
        telemetry.addData("collservo R", hw.rightCollectionServo.getPosition());

        hw.jewelsUp();
    }

    private void ReadDriverControls(Gamepad driver){
        xDrivePower = -driver.right_stick_x * DRIVE_POWER_SCALAR;
        yDrivePower = -driver.right_stick_y * DRIVE_POWER_SCALAR;
        anglePower = driver.left_stick_x * DRIVE_POWER_SCALAR;
        movementScalar = 1 - driver.right_trigger;
        if(movementScalar < 0.1) movementScalar = 0.1;
        rotationScalar = 1 - driver.left_trigger;
        if(rotationScalar < 0.1) rotationScalar = 0.1;

        telemetry.addData(movementScalar + "", rotationScalar);
    }

    private void ReadOperatorControls(Gamepad operator){
        collectorLeft = -1 * operator.left_stick_y * COLLECTOR_POWER_LEFT_SCALAR;
        collectorRight = -1 * operator.right_stick_y * COLLECTOR_POWER_RIGHT_SCALAR;
        conveyorForward = !operator.b;
        collectionUp = operator.y;
        maxConveyorPower = operator.a;
        flipperPosition = operator.left_trigger;
    }

    private void ComputePLoop() { //Function to control the robot's movements, this includes calculating P and calculating the motor powers
        if(USE_PID) {
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
        }

        telemetry.addData("Movement", "(" + xDrivePower + ", " + yDrivePower + ")");

        flPower = (yDrivePower - xDrivePower) * movementScalar + (pPower + anglePower) * rotationScalar;
        frPower = (yDrivePower + xDrivePower) * movementScalar - (pPower + anglePower) * rotationScalar;
        blPower = (yDrivePower + xDrivePower) * movementScalar + (pPower + anglePower) * rotationScalar;
        brPower = (yDrivePower - xDrivePower) * movementScalar - (pPower + anglePower) * rotationScalar;
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
