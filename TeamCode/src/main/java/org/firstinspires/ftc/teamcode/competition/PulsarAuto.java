package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;
import org.firstinspires.ftc.teamcode.lib.VuforiaController;
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
            return 0.2;
            //return this == BLUE ? 0.2 : 0.55;
        }

        protected double getJewelDownPosition(StartPosition pos) {
            return this == BLUE ? 0.800 : 0.700;
        }

        protected double getJewelDownAltPosition(StartPosition pos) {
            return 0.60;
            //return this == BLUE ? 0.65 : 0.25;
        }

        protected double getJewelKnockOffAngle(StartPosition pos) {
            return 10;
            //return this == BLUE ? 10 : -10;
        }

        protected double getDistanceToClearStone(StartPosition pos) {
            return 5.0;
        }

        protected double getRotationToFaceCyptobox(StartPosition pos) {
            return 90;
        }
    }

    protected enum StartPosition {BACK, FRONT}

    protected enum TargetJewelPosition {FRONT, BACK, NONE}

    private StartPosition startPosition = getStartPosition();
    private Alliance alliance = getAlliance();

    private RelicRecoveryVuMark targetColumn;
    private VuforiaController vuforiaController;
    MecanumRobot robot;
    protected PulsarRobotHardware hw;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing For Auto!");
        telemetry.update();

        hw = new PulsarRobotHardware(hardwareMap, getAlliance());
        robot = new MecanumRobot(hw, this, telemetry);

        hw.initializePositions(telemetry);

        vuforiaController = new VuforiaController(hardwareMap);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        targetColumn = vuforiaController.detectColumn();
        telemetry.addData("column", targetColumn.toString());

        hw.jewelServo.setPosition(alliance.getJewelDownPosition(startPosition));

        telemetry.addLine("Jewel Lowered");
        telemetry.update();

        Thread.sleep(2000); // TODO: Tune Value

        knockOffJewel(detectJewel());

        if (isSimpleAuto()) {
            //simpleAutoParkInSafeZone();
        } else {
            scoreInCryptobox(targetColumn);
        }

        hw.intakeDown();

        while (opModeIsActive()) { idle(); } // This prevents the servos from ragdolling.
    }

    private TargetJewelPosition detectJewel() throws InterruptedException {
        hw.jewelServo.setPosition(alliance.getJewelDownPosition(startPosition));

        telemetry.addLine("Jewel Lowered");
        telemetry.update();

        Thread.sleep(250); // TODO: Tune Value

        TargetJewelPosition targetJewelPosition = alliance.getTargetJewel(startPosition, hw.jewelDetector);

        telemetry.addData("r", hw.jewelDetector.red());
        telemetry.addData("g", hw.jewelDetector.green());
        telemetry.addData("b", hw.jewelDetector.blue());
        telemetry.addData("saw jewelServo", targetJewelPosition.toString());

        if (targetJewelPosition == TargetJewelPosition.NONE) {
            telemetry.addLine("Couldn't see jewel, moving...");
            telemetry.update();
            hw.jewelServo.setPosition(alliance.getJewelDownAltPosition(startPosition));

            telemetry.addLine("Jewel re-lowered");
            telemetry.update();

            Thread.sleep(250); // TODO: Tune Value

            targetJewelPosition = alliance.getTargetJewel(startPosition, hw.jewelDetector);
        }

        return targetJewelPosition;
    }

    private void knockOffJewel(TargetJewelPosition targetJewelPosition) {
        if (targetJewelPosition == TargetJewelPosition.NONE) return;
        double scalar = targetJewelPosition == TargetJewelPosition.FRONT ? -1 : 1;
        //robot.turn(scalar * alliance.getJewelKnockOffAngle(startPosition), 2000);
        robot.turn(scalar * alliance.getJewelKnockOffAngle(startPosition), 2000, 0.2);
        if (targetColumn == RelicRecoveryVuMark.UNKNOWN) {
            targetColumn = vuforiaController.detectColumn();
            telemetry.addData("column take 2", targetColumn.toString());
            telemetry.update();
        }
        hw.jewelsUp();
        //robot.turn(scalar * -alliance.getJewelKnockOffAngle(startPosition), 2000);
        robot.turn(scalar * -alliance.getJewelKnockOffAngle(startPosition), 2000, 0.2);
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

    private void moveMillisSupertween(float speed, double angle, long time) {
        robot.setTweenTime(time/2);
        robot.moveStraightMillis(speed, angle, time);
        robot.setTweenTime(700);
    }

    private void scoreInCryptobox(RelicRecoveryVuMark column) throws InterruptedException {
        hw.jewelsUp();
        Thread.sleep(200);
        hw.conveyorDown();
        // robot.moveStraightMillis(1, 3 * Math.PI / 2, 1650);
        long baseMoveValue = alliance == Alliance.BLUE ? 2800 : 3800; // 2450 : 3800
        switch (column) {
            case LEFT:
                baseMoveValue += 0;
                break;
            case CENTER:
                baseMoveValue += 450;
                break;
            case RIGHT:
                baseMoveValue += 900;
                break;
        }
        moveMillisSupertween(0.7f, alliance == Alliance.BLUE ? 3*Math.PI/2 : Math.PI/2, baseMoveValue);

        hw.initializePositionsTeleop();

        hw.intakeDown();
        robot.turn(alliance.getRotationToFaceCyptobox(startPosition), 3000); // 3000
        robot.setTweenTime(100);
        robot.moveStraightMillis(0.7f, 3 * Math.PI / 2, 400);
        hw.conveyor.setPower(hw.CONVEYOR_SPEED);
        Thread.sleep(7500);
        //robot.moveStraightMillis(0.7f, Math.PI / 2, 1000);
        moveMillisSupertween(0.5f, Math.PI / 2, 2000);
        robot.setTweenTime(0);
        robot.moveStraightMillis(1, 3 * Math.PI / 2, 2000);
        robot.moveStraightMillis(1, Math.PI / 2, 200);
        hw.conveyor.setPower(0);
        robot.setTweenTime(100);

    }
}
