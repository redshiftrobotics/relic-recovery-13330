package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.redshiftrobotics.lib.*;

/**
 * Created by adam on 10/11/17.
 * Main class for dealing with movements (namely straight motions and turning) on a four-wheeled
 * mecanum chassis.
 */



public class MecanumRobot {

    // Our motors. Assumes a four motor standard mecanum chassis.
    public DcMotor frontLeft, frontRight, backLeft, backRight;


   // CoordinatePIDController xyController;

    // Our PID controller instance
    IMUPIDController imupidController;

    // An imu wrapper instance, used mainly for testing, but also additional
    // abstraction.
    IMUImpl imuImpl;

    // Allows for access to containing LinearOpMode so we can gracefully stop.
    LinearOpMode context;

    // We still need to debug!
    Telemetry tm;

    static float ANGLE_THRESHOLD = 0.1f;

    /**
     * Our tuning constants for the mecanum chassis.
     */
    static float P_TUNING = 150f, I_TUNING = 2.0e-4f, D_TUNING = 0f;

    // Simple debug enable/disable
    static boolean DEBUG = true;

    /** Primary Constructor
     *
     * @param fl Front left motor
     * @param fr Front right motor
     * @param bl Back left motor
     * @param br Back Right motor
     * @param imu and instance of the Adafruit BN0055 sensor class
     * @param context a reference to an instance of LinearOpmode, so we can detect if the opmode
     *                has been stopped by the driver station and break from our loops.
     * @param tm a reference to the main telemetry object, for debugging purposes.
     */
    public MecanumRobot(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, BNO055IMU imu, LinearOpMode context, Telemetry tm) {
        this.frontLeft = fl;
        this.frontRight = fr;
        this.backLeft = bl;
        this.backRight = br;

        imuImpl = new IMUImpl(imu);
    //    xyController = new CoordinatePIDController(detector);
        imupidController = new IMUPIDController(imuImpl);
        this.context = context;
        this.tm = tm;
    }

    //TODO: Delete if we really won't need the below

    // THIS WILL PROBABLY NOT BE USED THIS SEASON. We don't really need coordinate PID, since
    // the encoder idea is scrapped.
   /* public void MoveTo(float x, float y, float targetAngle, float speed, float xTolerance, float yTolerance, float timeout) {

        // Check for speed overflow. 
        if (speed > 1) speed = 1;
        if (speed < -1) speed = -1;

        xyController.setXYTarget(x, y);
        imupidController.setTarget(targetAngle);

        Vector2D movement = new Vector2D(x, y);

        double angle = movement.GetDirection();

        Vector2D velocity = new Vector2D(0, 0);
        velocity.SetPolar(speed, angle);

        double velocityXComponent = velocity.GetXComponent();
        double velocityYComponent = velocity.GetYComponent();

        long elapsedTime = 0;
        long startTime = System.currentTimeMillis();
        long loopTime = System.currentTimeMillis();


        while ((Math.abs(xyController.Px) >= xTolerance
                || Math.abs(xyController.Px) >= yTolerance) && elapsedTime <= timeout) {

            double[] correctionXY = xyController.calculatePID(loopTime/1000);
            double correctionAngular = imupidController.calculatePID(loopTime/1000);
            applyMotorPower(velocityXComponent, velocityYComponent, correctionXY[0], correctionXY[1], correctionAngular);

            long currSysTime = System.currentTimeMillis();
            elapsedTime = currSysTime - startTime;
            loopTime = currSysTime - loopTime;
        }
    }*/

    /**
     * Moves a mecanum chassis straight at a specific angle for a specific distance in centimeters.
     * Uses PID.
     * @param speed the speed (as a fraction out of 1) component of the velocity vector.
     * @param angle the direction of the velocity vector
     * @param timeout the timeout, when the function should stop.
     *                (used as a safeguard to prevent a rogue robot).
     * @param cmDistance the distance the chassis should move (converts to encoders based on
     *                   chassis wheel diameter)
     */
    public void MoveStraight(float speed, double angle, long timeout, double cmDistance) {

        // Clear out all past PID data.
        imupidController.clearData();

        // These tunings have been established through our PID testing, and are good for
        // straight motion.
        imupidController.setTuning(P_TUNING, I_TUNING, D_TUNING);

        // Good to have a debug mode to enable/disable telemetry
        if (DEBUG) {
            tm.addData("P: " + imupidController.P + " I: " + imupidController.I + " D: " + imupidController.D, "");
            tm.update();
        }

        // Check for speed overflow (motors can only be set between 1 and -1)
        if (speed > 1) speed = 1;
        if (speed < -1) speed = -1;


        // We will represent velocity as a vector.
        Vector2D velocity = new Vector2D(0, 0);

        // The vectory class requires that we ADD polar coordinates to a vector -- does
        // not allow polar coordinate initialization.
        velocity.SetPolar(speed, angle);

        // Split vector into x velocity and y velocity to move at the specified angle.
        double velocityXComponent = velocity.GetXComponent();
        double velocityYComponent = velocity.GetYComponent();

        // Timekeeping variables...


        long elapsedTime = 0;
        long startTime = System.currentTimeMillis();

        // loopTime = time between each loop iteration
        long loopTime = System.currentTimeMillis();

        // This Conversion class contains all the conversion factors we need to switch between
        // encoders and cm.

        int encoderDistance = Conversion.cmToEncoder(cmDistance);

        // Use front motor for encoder counting (this can easily be changed). Some motors
        // may be wired such that the encoders count down. To reduce that pain in the ass, use
        // encoders on a different motor. It really doesn't matter, there are minimal differences between
        // encoders on different motors.
        float currentEncoderRotation = frontLeft.getCurrentPosition();

        // We might not start at position 0...
        float targetEncoderRotation = currentEncoderRotation + encoderDistance;

        // We need to account for elapsed time, potential stop conditions from the opmode container,
        // and changes in encoder position.
        while (elapsedTime <= timeout && context.opModeIsActive() && frontLeft.getCurrentPosition() < targetEncoderRotation) {
            double correctionAngular = imupidController.calculatePID(loopTime/1000);

            if (DEBUG) {
               // tm.addData("P: " + imupidController.P + "I: " + imupidController.I + "D: " + imupidController.D, "");
                // tm.update();
                tm.addData(" Front Left Position: ", frontLeft.getCurrentPosition());
            }

            applyMotorPower(velocityXComponent, velocityYComponent, correctionAngular);

            // Update time...
            long currSysTime = System.currentTimeMillis();
            elapsedTime = currSysTime - startTime;
            loopTime = currSysTime - loopTime;
        }
    }

    /**
     * Turns the robot to a specified angle heading using PID
     * @param robotAngle the angle to which the robot should turn, with the robot's current position
     *                   as a reference point. For instance, if we want to turn 90 degrees, and
     *                   the robot is already 100 degrees from the IMU's reference point, our target
     *                   will be 190 relative to the imu, but +90 relative to the robot.
     * @param timeout similar to the straight function, this acts a fail safe, and prevents the robot
     *                from idling for too long if the angle tolerance is never quite reached.
     */

    public void Turn(float robotAngle, long timeout) {
        // Set tuning for turning
        imupidController.setTuning(P_TUNING, I_TUNING, D_TUNING);

        // Clear out past data.
        imupidController.clearData();



        if (DEBUG) {
            tm.addData("Target: ", imupidController.target);
            tm.update();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                tm.addData("Error!", "");
                tm.update();
            }
            imupidController.addTarget(robotAngle);
            tm.addData("New Target: ", imupidController.target);
            tm.update();
        }

        // Time accounting

        long elapsedTime = 0;
        long startTime = System.currentTimeMillis();
        long loopTime = System.currentTimeMillis();

        if (DEBUG) {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        // Simpler while condition because there is no need for encoders...
        while (elapsedTime <= timeout && context.opModeIsActive()) {
            double correctionAngular = imupidController.calculatePID(loopTime/1000);
            if (DEBUG) {
                tm.addData("P: ", imupidController.P);
                tm.update();
            }

            // Apply motor power.
            // We don't need any x or y components of velocity (since we're not moving translationally,
            // just the angular correction +
            // our power constant of 0.5 to make the motors run at a reasonable speed.
            applyMotorPower(0, 0, 0.5 + correctionAngular);
            long currSysTime = System.currentTimeMillis();
            elapsedTime = currSysTime - startTime;
            loopTime = currSysTime - loopTime;

            // Our tolerance condition. It is contained within the loop as opposed to being
            // in the while condition, because it requires at least one PID calculation to be
            // performed in order to check our current error (imupidController.P) against the
            // tolerance.
            if (withinThreshold(imupidController.P, ANGLE_THRESHOLD) && withinThreshold(imupidController.lastError, ANGLE_THRESHOLD)) {
                break;
            }
        }
    }


    /**
     * Takes in x, y, and angular components of movement, and adds/subtracts them from the
     * correct motors to move the chassis appropriately.
     *
     * @param velocityX the x (sideways or strafing) component of the chassis velocity
     *                  (if calculated properly will be less than 1, but doesn't bounds check.
     * @param velocityY the y (forward) component of chassis velocity.
     * @param correctionAngular the angular correction to be applied as calculated by PID.
     */
        void applyMotorPower(double velocityX, double velocityY,  double correctionAngular) {

            // Divide all corrections by 2000 to make sure we don't overflow. Not a good solution!!!

            /*
            Values are added and subtracted here based on the direction the wheels need to go on a
            mecanum chassis to perform specific movements. For instance, Y velocity is always added,
            because all motors go the same direction when moving forwards and backwards. Angular
            movement and strafing require changes, because different wheels must move in different
            directions to make the movement possible.
             */

            // When this helper method is used in a turning function, correctionAngular
            // should be added to a power constant in order to make the wheels turn at a reasonable
            //speed. For small corrections during straight movement though, this is unnecessary.

            double frontLeftPower = velocityY  - velocityX  - correctionAngular/2000;
            double frontRightPower = velocityY + velocityX  + correctionAngular/2000;
            double backRightPower = velocityY - velocityX + correctionAngular/2000;
            double backLeftPower = velocityY + velocityX - correctionAngular/2000;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        }

         // Simple helper method to improve readability.
        public boolean withinThreshold(float value, float tolerance) {
            return Math.abs(value) <= tolerance;
        }

        public void STOP() {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
}

