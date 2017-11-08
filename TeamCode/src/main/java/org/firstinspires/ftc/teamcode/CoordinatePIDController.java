package org.firstinspires.ftc.teamcode;

/**
 * Created by adam on 10/11/17.
 */
public class CoordinatePIDController {
    float Px = 0f, Ix = 0f, Dx = 0f, Py = 0f, Iy = 0f, Dy = 0f;

    float PCONST_X = 1f, ICONST_X = 1f, D_CONST_X = 1f;

    float P_CONST_Y = 1f, I_CONST_Y = 1f, D_CONST_Y = 1f;

    long dT;
    float lastErrorX = 0f, lastErrorY = 0f;

    float xTarget = 0f, yTarget = 0f;
    DistanceDetector detector;

    public CoordinatePIDController(DistanceDetector d) {
        this.detector = d;
    }

    // Set target X and Y Coordinates
    public void setXYTarget(float xT, float yT) {
        this.xTarget = xT;
        this.yTarget = yT;
    }

    public void calculateP() {
        lastErrorX = Px;
        lastErrorY = Py;

        Px = detector.currentXDisplacement() - xTarget;
        Py = detector.currentXDisplacement() - yTarget;
    }

    public void calculateI() {
        Ix += Px * dT;
        Iy += Py * dT;
    }

    public void calculateD() {
        Dx = (Px - lastErrorX) / dT;
        Dy = (Py - lastErrorY) / dT;
    }

    /**Calculates PID
     *
     * @param deltaTime the time since the last iteration of the loop
     * @return a float array containing the x and y pid correction
     */
    public double[] calculatePID(long deltaTime) {
        dT = deltaTime;
        calculateP();
        calculateI();
        calculateD();

        return new double[] {
                PCONST_X * Px + ICONST_X * Ix + D_CONST_X * Dx,
                P_CONST_Y * Py + I_CONST_Y * Iy + D_CONST_Y * Dy
        };
    }
}
