package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.lib.PulsarRobotHardware;
import org.redshiftrobotics.lib.vuforia.ColumnController;
import org.redshiftrobotics.lib.MecanumRobot;


abstract public class PulsarAuto extends LinearOpMode {
    abstract protected Alliance getAlliance();

    abstract protected StartPosition getStartPosition();

    protected enum Alliance {
        BLUE, RED;

        public TargetJewelPosition getTargetJewel(StartPosition pos, ColorSensor colorSensor) {
            int red = colorSensor.red();
            int blue = colorSensor.blue();

            Alliance frontJewel;

            if (red > 3 && red > blue) frontJewel = RED;
            else if (blue > 3 && blue > red) frontJewel = BLUE;
            else return TargetJewelPosition.NONE;

            if (frontJewel == this) return TargetJewelPosition.BACK;
            else return TargetJewelPosition.FRONT;
        }

        public double getJewelUpPosition(StartPosition pos) {
            return this == BLUE ? 0.2 : 0.55;
        }

        public double getJewelDownPosition(StartPosition pos) {
            return this == BLUE ? 0.785 : 0.235;
        }

        public double getJewelDownAltPosition(StartPosition pos) {
            return this == BLUE ? 0.75 : 0.25;
        }

        public double getFrontJewelKnockOffAngle(StartPosition pos) {
            return this == BLUE ? 10 : -10;
        }

        public double getDistanceToClearStone(StartPosition pos) {
            return 45.0;/*26.5;*/
        }

        public double getAngleToAlignWithCryptobox(StartPosition pos) {
            if (pos == StartPosition.FRONT) {
                if (this == BLUE) return -90;
                return 90;
            } else {
                return 0;
            }
        }

        public double getDistanceToAlignWithColumn(StartPosition pos, RelicRecoveryVuMark column) {
            column = RelicRecoveryVuMark.CENTER;
            if (this == BLUE) {
                if (pos == StartPosition.FRONT) {
                    switch (column) {
                        case LEFT:
                            return 2;
                        case CENTER:
                            return 10;
                        case RIGHT:
                            return 25;
                    }
                } else {
                    switch (column) {
                        case LEFT:
                            return 10;
                        case CENTER:
                            return 30;
                        case RIGHT:
                            return 50;
                    }
                }
            } else {
                if (pos == StartPosition.FRONT) {
                    switch (column) {
                        case RIGHT:
                            return 10;
                        case CENTER:
                            return 30;
                        case LEFT:
                            return 50;
                    }
                } else {
                    switch (column) {
                        case RIGHT:
                            return 10;
                        case CENTER:
                            return 30;
                        case LEFT:
                            return 50;
                    }
                }
            }
            return 0;
        }

        public double getAngleToFaceCryptobox(StartPosition pos) {
            return this == BLUE ? 90 : -90;
        }
    }

    protected enum StartPosition {BACK, FRONT}

    protected enum TargetJewelPosition {FRONT, BACK, NONE}

    protected Servo jewelServo;

    protected ColorSensor jewelDetector;

    protected DcMotor conveyor;

    protected StartPosition startPosition = getStartPosition();
    protected Alliance alliance = getAlliance();

    protected boolean PULSAR_SIMPLE_AUTO = false;

    MecanumRobot robot;
    protected PulsarRobotHardware hw;

    ColumnController columnController;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new PulsarRobotHardware(hardwareMap);
        columnController = new ColumnController(hardwareMap);
        //leftJewel = hardwareMap.servo.get("leftJewel");
        //rightJewel = hardwareMap.servo.get("rightJewel");
        //leftJewelDetector = hardwareMap.colorSensor.get("leftJewelDetector");
        //rightJewelDetector = hardwareMap.colorSensor.get("rightJewelDetector");
        //conveyor = hardwareMap.dcMotor.get("conveyor");

        //leftJewel.setPosition(Alliance.BLUE.getJewelUpPosition(startPosition));
        //rightJewel.setPosition(Alliance.RED.getJewelUpPosition(startPosition));

        jewelServo = (alliance == Alliance.BLUE) ? hw.leftJewelServo : hw.rightJewelServo;
        jewelDetector = (alliance == Alliance.BLUE) ? hw.leftJewelDetector : hw.rightJewelDetector;

        robot = new MecanumRobot(
                hw.frontLeft,
                hw.frontRight,
                hw.backLeft,
                hw.backRight,
                hw.imu,
                this,
                telemetry
        );

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();


        jewelServo.setPosition(alliance.getJewelDownPosition(startPosition));

        telemetry.addLine("Jewel Lowered");
        telemetry.update();

        Thread.sleep(1000);

        TargetJewelPosition targetJewelPosition = alliance.getTargetJewel(startPosition, jewelDetector);

        telemetry.addData("saw jewelServo", targetJewelPosition.toString());
        if (targetJewelPosition == TargetJewelPosition.NONE) {
            telemetry.update();
            telemetry.addLine("Got None, moving");
            jewelServo.setPosition(alliance.getJewelDownAltPosition(startPosition));

            telemetry.addLine("Jewel Lowered");
            telemetry.update();

            Thread.sleep(1000);

            targetJewelPosition = alliance.getTargetJewel(startPosition, jewelDetector);
        }
        telemetry.update();

        Thread.sleep(1000);

        switch (targetJewelPosition) {
            case FRONT:
                break; // We do this later
            case BACK:
                robot.turn(alliance.getFrontJewelKnockOffAngle(startPosition), 2000);
                //jewelServo.setPosition(alliance.getJewelUpPosition(startPosition));
                robot.turn(-alliance.getFrontJewelKnockOffAngle(startPosition), 2000);
                break;
            case NONE:
                // jewelServo.setPosition(alliance.getJewelUpPosition(startPosition));
                // Thread.sleep(1000);
                break;
        }

        // for reference: robot.moveStraight(1,  3*Math.PI/2, 2000, 10);
        robot.moveStraight(1, Math.PI / 2, 5000, alliance.getDistanceToClearStone(startPosition));
        Thread.sleep(1000);

        //jewelServo.setPosition(alliance.getJewelUpPosition(startPosition));
        //Thread.sleep(1000);

        if (PULSAR_SIMPLE_AUTO) {
            telemetry.addLine("Simple auto only, exiting.");
            telemetry.update();
            return;
        }

        telemetry.addData("angle", alliance.getAngleToAlignWithCryptobox(startPosition));
        telemetry.update();

        robot.turn(alliance.getAngleToAlignWithCryptobox(startPosition), 2000, 0.3);
        robot.STOP();
        Thread.sleep(1000);

        double distanceToAlignWithColumn = alliance.getDistanceToAlignWithColumn(startPosition, columnController.getColumn());
        double angle = Math.PI / 2.0;
        if (distanceToAlignWithColumn < 0) {
            distanceToAlignWithColumn = Math.abs(distanceToAlignWithColumn);
            angle = 3.0 * Math.PI / 2.0;
        }
        robot.moveStraight(0.3f, angle, 5000, distanceToAlignWithColumn);
        robot.STOP();
        Thread.sleep(1000);

        robot.turn(alliance.getAngleToFaceCryptobox(startPosition), 2000, 0.3);
        Thread.sleep(1000);

        robot.moveStraight(0.3f, Math.PI / 2.0, 5000, 15);

        //conveyor.setPower(1.0);
        robot.STOP();
        Thread.sleep(1000);
    }
}
