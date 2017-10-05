package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Duncan on 10/4/2017.
 */
@Autonomous(name="Pixy Cam Tracking")
public class PixyCamTrack extends LinearOpMode {

    I2cDeviceSynch pixy;
    byte[] readCache;

    int zeroCheck = 0;
    int start = 0;
    boolean found = false;

    int xCenter = 0;
    int yCenter = 0;
    int width = 0;
    int height = 0;

    static final int SCREEN_WIDTH = 320;
    static final int SCREEN_HEIGHT = 200;

    static final int STANDARD_WIDTH = SCREEN_WIDTH / 4;
    static final int STANDARD_HEIGHT = SCREEN_WIDTH / 4;

    float distance = 0;
    float offset = 0;
    static final float distanceMult = 0.01f;
    static final float offsetMult = 0.01f;

    DcMotor leftDrive;
    DcMotor rightDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        pixy = hardwareMap.i2cDeviceSynch.get("pixyCam");
        pixy.engage();

        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");

        telemetry.setAutoClear(true);

        waitForStart();

        while (opModeIsActive()) {
            readCache = pixy.read(0x54, 26);

            zeroCheck = 0;
            for(int i = 0; i< 10; i++){
                zeroCheck+=readCache[i];
                if(((readCache[i] < 0) ? readCache[i] + 256 : readCache[i])==85&&((readCache[i+1] < 0) ? readCache[i+1] + 256 : readCache[i+1])==170&&((readCache[i+2] < 0) ? readCache[i+2] + 256 : readCache[i+2])==85&&((readCache[i+3] < 0) ? readCache[i+3] + 256 : readCache[i+3])==170){
                    start = i;
                    found = true;
                }
            }

            if(zeroCheck!=0&&found){
                xCenter = (readCache[start+9]==0?0:256) + ((readCache[start+8] < 0) ? readCache[start+8] + 256 : readCache[start+8]);
                yCenter = (readCache[start+11]==0?0:256) + ((readCache[start+10] < 0) ? readCache[start+10] + 256 : readCache[start+10]);
                width = (readCache[start+13]==0?0:256) + ((readCache[start+12] < 0) ? readCache[start+12] + 256 : readCache[start+12]);
                height = (readCache[start+15]==0?0:256) + ((readCache[start+14] < 0) ? readCache[start+14] + 256 : readCache[start+14]);
            }
            found = false;


            offset = (SCREEN_WIDTH / 2) - xCenter;
            distance = ((STANDARD_WIDTH - width) + (STANDARD_HEIGHT - height))/2;

            leftDrive.setPower(Range.clip(distance * distanceMult + offset * offsetMult,-1,1));
            rightDrive.setPower(Range.clip(distance * distanceMult - offset * offsetMult,-1,1));

            telemetry.update();
        }
    }
}
