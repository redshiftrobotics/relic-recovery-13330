package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.TypeConversion;

import java.util.Locale;

/**
 * Created by Duncan on 10/1/2017.
 */
@I2cSensor(name = "CMUCam5 Pixy", description = "Pixy Cam sensor from Charmed Labs", xmlTag = "CMUcam5")
public class PixyCam extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create8bit(0x54);

    public PixyObject data;

    public enum Register{
        FIRST(0),
        SYNC(0x00),
        SYNC_UPPER(0x01),
        CHECKSUM(0x02),
        CHECKSUM_UPPER(0x03),
        SIGNATURE(0x04),
        SIGNATURE_UPPER(0x05),
        X_CENTER(0x06),
        X_CENTER_UPPER(0x07),
        Y_CENTER(0x08),
        Y_CENTER_UPPER(0x09),
        WIDTH(0x0A),
        WIDTH_UPPER(0x0B),
        HEIGHT(0x0C),
        HEIGHT_UPPER(0x0D),
        LAST(HEIGHT_UPPER.bVal);

        public int bVal;

        Register(int bVal){
            this.bVal = bVal;
        }
    }

    protected void setOptimalReadWindow(){
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected void writeShort(final Register reg, short value){
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg){
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    private void sendCommunication(){
        writeShort(Register.SYNC,(short)1);
    }

    public boolean checkZero(){
        int zeroCheck = getSyncRaw() + getChecksumRaw() + getSignatureRaw() + getXCenterRaw() + getYCenterRaw() + getHeightRaw() + getWidthRaw();
        return  zeroCheck != 0;
    }

    public short readPixy (){
        return TypeConversion.byteArrayToShort(deviceClient.read(0x54, 2));
    }

    public short getSyncRaw(){
        return readShort(Register.SYNC);
    }

    public short getChecksumRaw(){
        return  readShort(Register.CHECKSUM);
    }

    public short getSignatureRaw(){
        return  readShort(Register.SIGNATURE);
    }

    public short getXCenterRaw(){
        return  readShort(Register.X_CENTER);
    }

    public short getYCenterRaw(){
        return  readShort(Register.Y_CENTER);
    }

    public short getWidthRaw(){
        return  readShort(Register.WIDTH);
    }

    public short getHeightRaw(){
        return  readShort(Register.HEIGHT);
    }

    public double filterRaw(short dataRaw){
        // The first 3 bits are alert bits that we don't care about here. We need to force them to
        // be 0s or 1s if the number is positive or negative depending on the sign
        if((dataRaw & 0x1000) == 0x1000) // Negative
            dataRaw |= 0xE000;
        else // Positive
            dataRaw &= 0x1FFF;

        // Multiple by least significant bit (2^-4 = 1/16) to scale
        return  dataRaw;
    }

    public void updateData(){
        sendCommunication();
        data.sync = filterRaw(getSyncRaw());
        data.checksum = filterRaw(getChecksumRaw());
        data.signature = filterRaw(getSignatureRaw());
        data.xCenter = filterRaw(getXCenterRaw());
        data.yCenter = filterRaw(getYCenterRaw());
        data.width = filterRaw(getWidthRaw());
        data.height = filterRaw(getHeightRaw());
    }

    public PixyObject getData(){
        return data;
    }

    public PixyCam(I2cDeviceSynch deviceClient){
        super(deviceClient, true);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();

        data = new PixyObject();
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true; //Need to update soon to add actual check, when fully implemented
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return String.format(Locale.getDefault(), "CMUcam5 Pixy Current Sensor %s",
                new RobotUsbDevice.FirmwareVersion(0));
    }
}
