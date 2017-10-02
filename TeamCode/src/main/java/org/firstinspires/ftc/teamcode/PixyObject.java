package org.firstinspires.ftc.teamcode;

/**
 * Created by Duncan on 10/1/2017.
 */

public class PixyObject {
    public double sync = 0;
    public double signature = 0;
    public double xCenter = 0; //xCenter overall is 160
    public double yCenter = 0; //yCenter overall is 100
    public double width = 0; //width max is 320
    public double height = 0; //height max is 200

    void UpdateObject(byte[] cache){
        signature = cache[7] * 256 + ((cache[6] < 0) ? cache[6] + 256 : cache[6]);
        xCenter = 160 - (cache[9] * 256 + ((cache[8] < 0) ? cache[8] + 256 : cache[8]));
        yCenter = 100 - (cache[11] * 256 + ((cache[10] < 0) ? cache[10] + 256 : cache[10]));
        width = cache[13] * 256 + ((cache[12] < 0) ? cache[12] + 256 : cache[12]);
        height = cache[15] * 256 + ((cache[14] < 0) ? cache[14] + 256 : cache[14]);
    }
}
