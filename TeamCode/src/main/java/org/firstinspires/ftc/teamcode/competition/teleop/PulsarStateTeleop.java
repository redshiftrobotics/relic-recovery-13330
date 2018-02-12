package org.firstinspires.ftc.teamcode.competition.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.competition.auto.PulsarAuto;
import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;
import org.redshiftrobotics.lib.blockplacer.Col;

@TeleOp(name="Pulsar State Teleop", group="Pulsar")
public class PulsarStateTeleop extends LinearOpMode {
    private static final boolean RELIC = true; // TODO: actually map relicPower to a control

    private PulsarRobotHardware hw;

    private boolean hasHadInput = false;

    private static final double DRIVE_POWER_SCALAR = 1;
    private static final double COLLECTOR_POWER_LEFT_SCALAR = 1;
    private static final double COLLECTOR_POWER_RIGHT_SCALAR = 1;
    private static final double RELIC_POWER = 1;
    private static final double RELIC_DOWN_POSITION = 0.3;
    private static final double RELIC_UP_POSITION = 0.60;
    private static final double RELIC_OPEN_POSITION = 1;
    private static final double RELIC_CLOSED_POSITION = 0;
    private static final double RELIC_MAX_POSITION = Double.POSITIVE_INFINITY; // TUNE

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
    private boolean forceAlignWithCrytobox = false;
    private boolean relicOpen = false;
    private Col cryptoboxColumn = null;

    private double flipperPosition = 0;
    private double movementScalar = 0;
    private double rotationScalar = 0;
    private double relicWrist = 0;

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

        hw = new PulsarRobotHardware(this, PulsarAuto.Alliance.BLUE); // TODO: we actually need this now

        telemetry.addLine("Ready for TeleOp (at State)!");
        telemetry.addLine();
        telemetry.addLine("Press START to Begin!");
        telemetry.update();

        waitForStart();

        hw.jewelsUp(false); // Don't sleep, we want to be able to start ASAP

        while (opModeIsActive()) {
            CheckForInputPresence();
            ReadDriverControls(gamepad1);
            ReadOperatorControls(gamepad2);
            ComputePLoop();
            NormalizeMotorPowers();
            //Update all hardware based on user input, these must be called last
            UpdateMotors();
            UpdateServos();
            telemetry.addData("Relic Enc", hw.motors.relic.getCurrentPosition());
            telemetry.update();
        }
    }

    private void CheckForInputPresence() {
        hasHadInput = hasHadInput || !gamepad1.atRest() || !gamepad2.atRest();
        hasHadInput = true; // FIXME
    }

    private void UpdateMotors(){
        if (!hasHadInput) return;

        if (forceAlignWithCrytobox && cryptoboxColumn != null) {
            telemetry.addLine("MOTORS DISABLED-- Aligning with Cryptobox [EXPERIMENTAL]");
            telemetry.addData("Cryptobox Column", cryptoboxColumn);
            hw.alignWithCryptobox(cryptoboxColumn);
        } else {
            hw.motors.frontLeft.setPower(flPower);
            hw.motors.frontRight.setPower(frPower);
            hw.motors.backLeft.setPower(blPower);
            hw.motors.backRight.setPower(brPower);
        }

        hw.motors.leftCollection.setPower(collectorLeft);
        hw.motors.rightCollection.setPower(collectorRight);

        double rawConveyorSpeed = maxConveyorPower ? 1 : hw.CONVEYOR_POWER;
        if (!conveyorForward) rawConveyorSpeed *= -1;
        hw.setConveyorPower(rawConveyorSpeed);

        if (RELIC) {
            if (hw.motors.relic.getCurrentPosition() >= RELIC_MAX_POSITION || hw.motors.relic.getCurrentPosition() <= 0) {
                telemetry.addLine("Relic at min/max position!");
                hw.motors.relic.setPower(0);
            } else {
                hw.motors.relic.setPower(relicPower);
            }
        }
    }

    private void UpdateServos(){
        hw.setFlipperPosition(flipperPosition);

        if (collectionUp) hw.collectorUp();
        else hw.collectorDown();

        hw.servos.relicClaw.setPosition(relicOpen ? RELIC_OPEN_POSITION : RELIC_CLOSED_POSITION);
        hw.servos.relicWrist.setPosition(relicWrist);

        hw.jewelsUp(false);
    }

    private void ReadDriverControls(Gamepad driver){
        xDrivePower = -driver.right_stick_x * DRIVE_POWER_SCALAR;
        yDrivePower = -driver.right_stick_y * DRIVE_POWER_SCALAR;
        anglePower = driver.left_stick_x * DRIVE_POWER_SCALAR;

        movementScalar = 1 - driver.right_trigger;
        if(movementScalar < 0.1) movementScalar = 0.1;

        rotationScalar = 1 - driver.left_trigger;
        if(rotationScalar < 0.1) rotationScalar = 0.1;

        if (driver.dpad_up || driver.dpad_left || driver.dpad_right) {
            forceAlignWithCrytobox = true;
            if (driver.dpad_up) cryptoboxColumn = Col.CENTER;
            else if (driver.dpad_left) cryptoboxColumn = Col.LEFT;
            else if (driver.dpad_right) cryptoboxColumn = Col.RIGHT;
        } else {
            forceAlignWithCrytobox = false;
            cryptoboxColumn = null;
        }

        telemetry.addData("Movement Scalar", movementScalar);
        telemetry.addData("Rotation Scalar", rotationScalar);
    }

    private void ReadOperatorControls(Gamepad operator){
        collectorLeft = -operator.left_stick_y * COLLECTOR_POWER_LEFT_SCALAR;
        collectorRight = -operator.right_stick_y * COLLECTOR_POWER_RIGHT_SCALAR;

        conveyorForward = !operator.b;
        collectionUp = operator.y;
        maxConveyorPower = operator.a;
        flipperPosition = operator.left_trigger;

        relicWrist = operator.right_trigger > 0.5 ? RELIC_UP_POSITION : RELIC_DOWN_POSITION;
        if (operator.dpad_up) relicPower = RELIC_POWER;
        else if (operator.dpad_down) relicPower = -RELIC_POWER;
        else relicPower = 0;
        relicOpen = operator.dpad_left;
    }

    private void ComputePLoop() {
        telemetry.addData("Movement", "(" + xDrivePower + ", " + yDrivePower + ")");

        flPower = (yDrivePower - xDrivePower) * movementScalar + (pPower + anglePower) * rotationScalar;
        frPower = (yDrivePower + xDrivePower) * movementScalar - (pPower + anglePower) * rotationScalar;
        blPower = (yDrivePower + xDrivePower) * movementScalar + (pPower + anglePower) * rotationScalar;
        brPower = (yDrivePower - xDrivePower) * movementScalar - (pPower + anglePower) * rotationScalar;
    }

    private void NormalizeMotorPowers() {
        // Calculate which motor has the highest non-normalized power
        maxPower = Math.abs(flPower);
        if (Math.abs(frPower) > maxPower) maxPower = Math.abs(frPower);
        else if (Math.abs(blPower) > maxPower) maxPower = Math.abs(blPower);
        else if (Math.abs(brPower) > maxPower) maxPower = Math.abs(brPower);

        if(maxPower<1) maxPower = 1;

        // Scale all motor power to fit within -1.0 to 1.0 based on maxPower
        flPower = flPower / maxPower;
        frPower = frPower / maxPower;
        blPower = blPower / maxPower;
        brPower = brPower / maxPower;
    }
}
