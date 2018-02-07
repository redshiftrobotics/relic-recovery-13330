package org.firstinspires.ftc.teamcode.competition.auto.simple;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.competition.auto.PulsarAuto;

@Autonomous(name = "Red A (Simple)", group = "Final Auto Simple")
public class PulsarSimpleAutoRedA extends PulsarAuto {
    @Override
    protected Alliance getAlliance() { return Alliance.RED; }

    @Override
    protected StartPosition getStartPosition() { return StartPosition.A; }

    @Override
    protected boolean isSimpleAuto() { return true; }
}
