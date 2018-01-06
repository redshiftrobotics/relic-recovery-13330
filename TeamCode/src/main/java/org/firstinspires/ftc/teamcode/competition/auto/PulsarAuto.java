package org.firstinspires.ftc.teamcode.competition.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;
import org.redshiftrobotics.lib.vuforia.VuforiaController;
import org.redshiftrobotics.lib.pid.StraightPIDController;
import org.redshiftrobotics.lib.pid.TurningPIDController;


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

        protected long getDistanceToClearStone(StartPosition pos) {
            return 5;
        }

        protected double getRotationToFaceCyptobox(StartPosition pos) {
            return 90;
        }
    }

    protected enum StartPosition {BACK, FRONT}

    protected enum TargetJewelPosition {FRONT, BACK, NONE}

    private StartPosition startPosition = getStartPosition();
    private PulsarAuto.Alliance alliance = getAlliance();

    private RelicRecoveryVuMark targetColumn;
    private VuforiaController vuforiaController;
    protected PulsarRobotHardware hw;
    protected StraightPIDController straightPIDController;
    protected TurningPIDController turningPIDController;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing For Auto!");
        telemetry.update();

        hw = new PulsarRobotHardware(this, getAlliance());
        hw.initializePositions(telemetry);

        vuforiaController = new VuforiaController(hw);

        straightPIDController = new StraightPIDController(hw);
        turningPIDController = new TurningPIDController(hw);

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
            simpleAutoParkInSafeZone();
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
        turningPIDController.turn(scalar * alliance.getJewelKnockOffAngle(startPosition), 2000, 0.2);
        if (targetColumn == RelicRecoveryVuMark.UNKNOWN) {
            targetColumn = vuforiaController.detectColumn();
            telemetry.addData("column take 2", targetColumn.toString());
            telemetry.update();
        }
        hw.jewelsUp();
        //robot.turn(scalar * -alliance.getJewelKnockOffAngle(startPosition), 2000);
        turningPIDController.turn(scalar * -alliance.getJewelKnockOffAngle(startPosition), 2000, 0.2);
    }

    private void simpleAutoParkInSafeZone() {
        if (!isSimpleAuto()) throw new IllegalStateException("Attempted to run simple auto in non-simple auto!");
        double angle = 3 * Math.PI / 2;
        if ((alliance == Alliance.RED && startPosition == StartPosition.FRONT) || (alliance == Alliance.BLUE && startPosition == StartPosition.BACK)) {
            turningPIDController.turn(Math.PI / 5, 1600);
        } else {
            turningPIDController.turn(-Math.PI / 5, 1600);
        }

        hw.jewelsUp();

        straightPIDController.moveStraight(1, angle, 1500, alliance.getDistanceToClearStone(startPosition));

        telemetry.addLine("Simple auto only, exiting.");
        telemetry.update();
    }

    private void scoreInCryptobox(RelicRecoveryVuMark column) throws InterruptedException {
        hw.jewelsUp();
        hw.conveyorDown();
        Thread.sleep(200);
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
        straightPIDController.moveStraight(0.7f, alliance == Alliance.BLUE ? 3*Math.PI/2 : Math.PI/2, baseMoveValue);

        hw.initializePositionsTeleop();

        hw.intakeDown();
        turningPIDController.turn(alliance.getRotationToFaceCyptobox(startPosition), 3000);
        straightPIDController.moveStraight(0.7f, 3 * Math.PI / 2, 400, 100);
        hw.conveyor.setPower(hw.CONVEYOR_SPEED);
        Thread.sleep(7500);
        straightPIDController.moveStraight(0.5f, Math.PI / 2, 2000);
        straightPIDController.moveStraight(1, 3 * Math.PI / 2, 2000);
        straightPIDController.moveStraight(1, Math.PI / 2, 200);
        hw.conveyor.setPower(0);
    }
}
