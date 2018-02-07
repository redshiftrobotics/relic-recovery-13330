package org.firstinspires.ftc.teamcode.competition.auto.full;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.competition.auto.PulsarAuto;

@Autonomous(name = "Red A", group = "Final Auto")
public class PulsarAutoRedA extends PulsarAuto {
    @Override
    protected Alliance getAlliance() { return Alliance.RED; }

    @Override
    protected StartPosition getStartPosition() { return StartPosition.A; }
}
