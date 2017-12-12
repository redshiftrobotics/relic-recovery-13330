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
            return this == BLUE ? 0.75 : 0.25;
        }

        protected double getJewelKnockOffAngle(StartPosition pos) {
            return this == BLUE ? 10 : -10;
        }

        protected double getDistanceToClearStone(StartPosition pos) {
            return 5.0;/*26.5;*/
        }

        public double getAngleToAlignWithCryptobox(StartPosition pos) {
            if (pos == StartPosition.FRONT) {
                if (this == BLUE) return -90;
                return 90;
            } else {
                return 0;
            }
        }

        protected double getDistanceToAlignWithColumn(StartPosition pos, RelicRecoveryVuMark column) {
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

        protected double getAngleToFaceCryptobox(StartPosition pos) {
            return this == BLUE ? 90 : -90;
        }
    }

    protected enum StartPosition {BACK, FRONT}

    protected enum TargetJewelPosition {FRONT, BACK, NONE}

    protected DcMotor conveyor;

    protected StartPosition startPosition = getStartPosition();
    protected Alliance alliance = getAlliance();

    protected boolean PULSAR_SIMPLE_AUTO = false;

    MecanumRobot robot;
    protected PulsarRobotHardware hw;

    ColumnController columnController;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new PulsarRobotHardware(hardwareMap, getAlliance());
        telemetry.addLine("Initializing For Auto!");
        telemetry.update();

        robot = new MecanumRobot(hw, this, telemetry);


        /**
         * Initialization Values
         */

        hw.initializePositions(telemetry);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();


        hw.jewelServo.setPosition(alliance.getJewelDownPosition(startPosition));

        telemetry.addLine("Jewel Lowered");
        telemetry.update();

        Thread.sleep(1000);

        TargetJewelPosition targetJewelPosition = alliance.getTargetJewel(startPosition, hw.jewelDetector);


        telemetry.addData("r", hw.jewelDetector.red());
        telemetry.addData("g", hw.jewelDetector.green());
        telemetry.addData("b", hw.jewelDetector.blue());
        telemetry.addData("saw jewelServo", targetJewelPosition.toString());
        if (targetJewelPosition == TargetJewelPosition.NONE) {
            telemetry.update();
            telemetry.addLine("Got None, moving");
            hw.jewelServo.setPosition(alliance.getJewelDownAltPosition(startPosition));

            telemetry.addLine("Jewel Lowered");
            telemetry.update();

            Thread.sleep(1000);

            targetJewelPosition = alliance.getTargetJewel(startPosition, hw.jewelDetector);
        }
        telemetry.update();

        Thread.sleep(1000);

        switch (targetJewelPosition) {
            case FRONT:
                robot.turn(-alliance.getJewelKnockOffAngle(startPosition), 2000);
                hw.leftJewelServo.setPosition(0.3);
                hw.rightJewelServo.setPosition(0.9);
                robot.turn(alliance.getJewelKnockOffAngle(startPosition), 2000);
                break;
            case BACK:
                robot.turn(alliance.getJewelKnockOffAngle(startPosition), 2000);
                hw.leftJewelServo.setPosition(0.3);
                hw.rightJewelServo.setPosition(0.9);
                robot.turn(-alliance.getJewelKnockOffAngle(startPosition), 2000);
                break;
            case NONE:
                // Thread.sleep(1000);
                break;
        }

        hw.jewelServo.setPosition(alliance.getJewelUpPosition(startPosition));

        if (isSimpleAuto()) {
            double angle = 3 * Math.PI / 2;
            if ((alliance == Alliance.RED && startPosition == StartPosition.FRONT) || (alliance == Alliance.BLUE && startPosition == StartPosition.BACK)) {
                robot.turn(Math.PI / 5, 1600);
            } else {
                robot.turn(-Math.PI / 5, 1600);
            }
            robot.moveStraight(1, angle, 1500, alliance.getDistanceToClearStone(startPosition));
            Thread.sleep(1000);

            hw.leftJewelServo.setPosition(0.3);
            hw.rightJewelServo.setPosition(0.9);
            //Thread.sleep(1000);

            telemetry.addLine("Simple auto only, exiting.");
            telemetry.update();
            return;
        }

        robot.moveStraight(1, 3 * Math.PI / 2, 5000, alliance.getDistanceToClearStone(startPosition));
        Thread.sleep(1000);

        moveForwardOneCryptoboxColumn();
        moveForwardOneCryptoboxColumn();


        /*robot.turn(alliance.getAngleToAlignWithCryptobox(startPosition), 2000, 0.3);
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
        Thread.sleep(1000);*/
    }

    private void moveForwardOneCryptoboxColumn() throws InterruptedException {
        hw.jewelServo.setPosition(alliance.getJewelDownAltPosition(startPosition));
        Thread.sleep(1000);

        //robot.moveTillOpticalDistanceSensor(1, 3 * Math.PI / 2, 5000, 2);
        robot.moveStraightMillis(1,3 * Math.PI / 2, 1000);
        Thread.sleep(1000);

        hw.jewelServo.setPosition(alliance.getJewelUpPosition(startPosition));
        Thread.sleep(1000);

        robot.moveStraight(0.5f, 3 * Math.PI / 2, 5000, 15);
        Thread.sleep(1000);
    }
}
