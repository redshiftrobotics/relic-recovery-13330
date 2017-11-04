package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Back", group = "000 Final Auto")
public class PulsarAutoRedBack extends PulsarAuto {
    @Override
    protected Alliance getAlliance() { return Alliance.RED; }

    @Override
    protected StartPosition getStartPosition() { return StartPosition.BACK; }
}
