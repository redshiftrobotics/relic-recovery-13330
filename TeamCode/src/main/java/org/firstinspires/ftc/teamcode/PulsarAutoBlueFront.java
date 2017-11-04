package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Front", group = "000 Final Auto")
public class PulsarAutoBlueFront extends PulsarAuto {
    @Override
    protected Alliance getAlliance() { return Alliance.BLUE; }

    @Override
    protected StartPosition getStartPosition() { return StartPosition.FRONT; }
}
