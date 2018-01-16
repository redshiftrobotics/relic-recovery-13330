package org.firstinspires.ftc.teamcode.competition.auto.simple;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.competition.auto.PulsarAuto;

@Autonomous(name = "Blue Back (Simple)", group = "Final Auto Simple")
public class PulsarSimpleAutoBlueBack extends PulsarAuto {
    @Override
    protected Alliance getAlliance() { return Alliance.BLUE; }

    @Override
    protected StartPosition getStartPosition() { return StartPosition.BACK; }

    @Override
    protected boolean isSimpleAuto() { return true; }
}
