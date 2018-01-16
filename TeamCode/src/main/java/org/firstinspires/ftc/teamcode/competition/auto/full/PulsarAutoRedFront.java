package org.firstinspires.ftc.teamcode.competition.auto.full;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.competition.auto.PulsarAuto;

@Autonomous(name = "Red Front", group = "Final Auto")
public class PulsarAutoRedFront extends PulsarAuto {
    @Override
    protected Alliance getAlliance() { return Alliance.RED; }

    @Override
    protected StartPosition getStartPosition() { return StartPosition.FRONT; }
}
