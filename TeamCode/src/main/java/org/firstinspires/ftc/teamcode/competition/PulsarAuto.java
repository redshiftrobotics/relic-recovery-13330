package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;
import org.redshiftrobotics.lib.vuforia.ColumnController;
import org.firstinspires.ftc.teamcode.lib.MecanumRobot;


abstract public class PulsarAuto extends LinearOpMode {
    abstract protected Alliance getAlliance();
    abstract protected StartPosition getStartPosition();
    protected boolean isSimpleAuto() { return false; }

    public enum Alliance {
        BLUE, RED;

        protected TargetJewelPosition getTargetJewel(StartPosition pos, ColorSensor colorSensor) {
            int red = colorSensor.red();
            int blue = colorSensor.blue();

            Alliance backJewel;

            if (red > blue) backJewel = RED;
            else if (blue > red) backJewel = BLUE;
            else return TargetJewelPosition.NONE;
            //old: back:front; new: front:back
            if (backJewel == this) return TargetJewelPosition.FRONT;
            else return TargetJewelPosition.BACK;
        }

        protected double getJewelUpPosition(StartPosition pos) {
            return this == BLUE ? 0.2 : 0.55;
        }

        protected double getJewelDownPosition(StartPosition pos) {
            return this == BLUE ? 0.800 : 0.200;
        }

        protected double getJewelDownAltPosition(StartPosition pos) {
            return this == BLUE ? 0.65 : 0.25;
        }

        protected double getJewelKnockOffAngle(StartPosition pos) {
            return this == BLUE ? 10 : -10;
        }

        protected double getDistanceToClearStone(StartPosition pos) {
            return 5.0;
        }
    }

    protected enum StartPosition {BACK, FRONT}

    protected enum TargetJewelPosition {FRONT, BACK, NONE}

    private StartPosition startPosition = getStartPosition();
    private Alliance alliance = getAlliance();

    MecanumRobot robot;
    protected PulsarRobotHardware hw;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing For Auto!");
        telemetry.update();

        hw = new PulsarRobotHardware(hardwareMap, getAlliance());
        robot = new MecanumRobot(hw, this, telemetry);

        hw.initializePositions(telemetry);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        hw.jewelServo.setPosition(alliance.getJewelDownPosition(startPosition));

        telemetry.addLine("Jewel Lowered");
        telemetry.update();

        Thread.sleep(1000); // TODO: Tune Value

        knockOffJewel(detectJewel());

        if (isSimpleAuto()) {
            simpleAutoParkInSafeZone();
        } else {
            scoreInCryptobox();
        }

        while (opModeIsActive()) { idle(); } // This prevents the servos from ragdolling.
    }

    private TargetJewelPosition detectJewel() throws InterruptedException {
        hw.jewelServo.setPosition(alliance.getJewelDownPosition(startPosition));

        telemetry.addLine("Jewel Lowered");
        telemetry.update();

        Thread.sleep(1000); // TODO: Tune Value

        TargetJewelPosition targetJewelPosition = alliance.getTargetJewel(startPosition, hw.jewelDetector);

        telemetry.addData("r", hw.jewelDetector.red());
        telemetry.addData("g", hw.jewelDetector.green());
        telemetry.addData("b", hw.jewelDetector.blue());
        telemetry.addData("saw jewelServo", targetJewelPosition.toString());
        telemetry.update();

        if (targetJewelPosition == TargetJewelPosition.NONE) {
            telemetry.addLine("Couldn't see jewel, moving...");
            telemetry.update();
            hw.jewelServo.setPosition(alliance.getJewelDownAltPosition(startPosition));

            telemetry.addLine("Jewel re-lowered");
            telemetry.update();

            Thread.sleep(1000); // TODO: Tune Value

            targetJewelPosition = alliance.getTargetJewel(startPosition, hw.jewelDetector);
        }

        return targetJewelPosition;
    }

    private void knockOffJewel(TargetJewelPosition targetJewelPosition) {
        if (targetJewelPosition == TargetJewelPosition.NONE) return;
        double scalar = targetJewelPosition == TargetJewelPosition.FRONT ? -1 : 1;
        robot.turn(scalar * alliance.getJewelKnockOffAngle(startPosition), 2000);
        hw.jewelsUp();
        robot.turn(scalar * -alliance.getJewelKnockOffAngle(startPosition), 2000);
    }

    private void simpleAutoParkInSafeZone() {
        if (!isSimpleAuto()) throw new IllegalStateException("Attempted to run simple auto in non-simple auto!");
        double angle = 3 * Math.PI / 2;
        if ((alliance == Alliance.RED && startPosition == StartPosition.FRONT) || (alliance == Alliance.BLUE && startPosition == StartPosition.BACK)) {
            robot.turn(Math.PI / 5, 1600);
        } else {
            robot.turn(-Math.PI / 5, 1600);
        }

        hw.jewelsUp();

        robot.moveStraight(1, angle, 1500, alliance.getDistanceToClearStone(startPosition));

        telemetry.addLine("Simple auto only, exiting.");
        telemetry.update();
    }

    private void scoreInCryptobox() {
        robot.moveStraightMillis(1, 3 * Math.PI / 2, 1550);

        hw.initializePositionsTeleop();
        hw.conveyorDown();

        robot.turn(85, 3000);
        robot.setTweenTime(100);
        robot.moveStraightMillis(0.7f, 3 * Math.PI / 2, 400);

        hw.conveyor.setPower(hw.CONVEYOR_SPEED);
    }
}
