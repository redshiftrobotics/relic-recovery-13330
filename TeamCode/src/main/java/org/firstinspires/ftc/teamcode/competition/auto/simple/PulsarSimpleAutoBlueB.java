package org.firstinspires.ftc.teamcode.competition.auto.simple;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.competition.auto.PulsarAuto;

@Autonomous(name = "Blue B (Simple)", group = "Final Auto Simple")
public class PulsarSimpleAutoBlueB extends PulsarAuto {
    @Override
    protected Alliance getAlliance() { return Alliance.BLUE; }

    @Override
    protected StartPosition getStartPosition() { return StartPosition.B; }

    @Override
    protected boolean isSimpleAuto() { return true; }
}
