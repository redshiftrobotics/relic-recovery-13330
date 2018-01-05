package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Back (Simple)", group = "Final Auto Simple")
public class PulsarSimpleAutoBlueBack extends PulsarAuto {
    @Override
    protected Alliance getAlliance() { return Alliance.BLUE; }

    @Override
    protected StartPosition getStartPosition() { return StartPosition.BACK; }

    @Override
    protected boolean isSimpleAuto() { return true; }
}
