package org.firstinspires.ftc.teamcode.lib;

/**
 * Created by adam on 10/25/17.
 */
public class EncoderDistanceConverter {
    public static int FULL_ENCODER_ROTATION = 1440;

    // Wheel diameter of Andy mark mecanum wheels. Don't know the exact model number, but they
    // are colloquially known as the "crappy" mecanum wheels.
    public static float WHEEL_DIAMETER_CM = 10.16f;


    // Decent estimate of the wheel circumerefence.
    public static double WHEEL_CIRCUMFERENCE_CM = Math.PI * WHEEL_DIAMETER_CM;

    public static int cmToEncoder(double cm) {
        return (int) Math.round(cm / WHEEL_CIRCUMFERENCE_CM * FULL_ENCODER_ROTATION);
    }

    public static double encoderToCM(int encoderCount) {
        return encoderCount / FULL_ENCODER_ROTATION * WHEEL_CIRCUMFERENCE_CM;
    }
}
