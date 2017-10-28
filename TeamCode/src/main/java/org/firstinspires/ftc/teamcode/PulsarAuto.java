package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


abstract public class PulsarAuto extends LinearOpMode {
    abstract protected Alliance getAlliance();
    abstract protected StartPosition getStartPosition();

    protected enum Alliance {
        BLUE, RED;

        public TargetJewelPosition getTargetJewel(StartPosition pos, ColorSensor colorSensor) {
            int red = colorSensor.red();
            int blue = colorSensor.blue();

            Alliance backJewel;

            if (red > 3 && red > blue) backJewel = RED;
            else if (blue > 3 && blue > red) backJewel = BLUE;
            else return TargetJewelPosition.NONE;

            if (backJewel == this) return TargetJewelPosition.FRONT;
            else return TargetJewelPosition.BACK;
        }

        public double getJewelUpPosition(StartPosition pos) { return this == BLUE ? 0 : 0; }
        public double getJewelDownPosition(StartPosition pos) { return this == BLUE ? 0 : 0; }
        public double getFrontJewelKnockOffAngle(StartPosition pos) { return this == BLUE ? -30 : 30; }
        public double getDistanceToClearStone(StartPosition pos) { return pos == StartPosition.FRONT ? 80 : 60; }
        public double getAngleToAlignWithCryptobox(StartPosition pos) { return pos == StartPosition.FRONT ? 90 : 0; }
        public double getDistanceToAlignWithColumn(StartPosition pos, RelicRecoveryVuMark column) {
            if (this == BLUE) {
                if (pos == StartPosition.FRONT) {
                    switch (column) {
                        case LEFT: return 10;
                        case CENTER: return 30;
                        case RIGHT: return 50;
                    }
                } else {
                    switch (column) {
                        case LEFT: return 10;
                        case CENTER: return 30;
                        case RIGHT: return 50;
                    }
                }
            } else {
                if (pos == StartPosition.FRONT) {
                    switch (column) {
                        case RIGHT: return 10;
                        case CENTER: return 30;
                        case LEFT: return 50;
                    }
                } else {
                    switch (column) {
                        case RIGHT: return 10;
                        case CENTER: return 30;
                        case LEFT: return 50;
                    }
                }
            }
        }
        public double getAngleToFaceCryptobox(StartPosition pos) { return this == BLUE ? -90 : 90; }
    }
    protected enum StartPosition { BACK, FRONT }

    private enum TargetJewelPosition { FRONT, BACK, NONE }

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Servo jewel;
    private Servo leftJewel;
    private Servo rightJewel;

    private ColorSensor jewelDetector;
    private ColorSensor leftJewelDetector;
    private ColorSensor rightJewelDetector;

    private DcMotor conveyor;

    private StartPosition startPosition = getStartPosition();
    private Alliance alliance = getAlliance();

    private MecanumRobot robot;

    private ColumnController columnController;

    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        columnController = new ColumnController(hardwareMap);
        leftJewel = hardwareMap.servo.get("leftJewel");
        rightJewel = hardwareMap.servo.get("rightJewel");
        leftJewelDetector = hardwareMap.colorSensor.get("leftJewelDetector");
        rightJewelDetector = hardwareMap.colorSensor.get("rightJewelDetector");
        conveyor = hardwareMap.dcMotor.get("conveyor");

        leftJewel.setPosition(Alliance.BLUE.getJewelUpPosition(startPosition));
        rightJewel.setPosition(Alliance.RED.getJewelUpPosition(startPosition));

        jewel = (alliance == Alliance.BLUE) ? leftJewel : rightJewel;
        jewelDetector = (alliance == Alliance.BLUE) ? leftJewelDetector : rightJewelDetector;

        robot = new MecanumRobot(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                imu,
                this,
                telemetry
        );
        waitForStart();

        jewel.setPosition(alliance.getJewelDownPosition(startPosition));

        TargetJewelPosition targetJewelPosition = alliance.getTargetJewel(startPosition, jewelDetector);

        switch (targetJewelPosition) {
            case FRONT: break; // We do this later
            case BACK:
                robot.turn(alliance.getFrontJewelKnockOffAngle(startPosition), 2000);
                robot.turn(-alliance.getFrontJewelKnockOffAngle(startPosition), 2000);
                // fallthrough
            case NONE:
                jewel.setPosition(alliance.getJewelUpPosition(startPosition));
                break;
        }

        robot.moveStraight(1, 5000, alliance.getDistanceToClearStone(startPosition));

        jewel.setPosition(alliance.getJewelUpPosition(startPosition));

        robot.turn(alliance.getAngleToAlignWithCryptobox(startPosition), 2000);

        robot.moveStraight(1, 5000, alliance.getDistanceToAlignWithColumn(startPosition, columnController.getColumn()));

        robot.turn(alliance.getAngleToFaceCryptobox(startPosition), 2000);

        conveyor.setPower(1.0);
    }
}
