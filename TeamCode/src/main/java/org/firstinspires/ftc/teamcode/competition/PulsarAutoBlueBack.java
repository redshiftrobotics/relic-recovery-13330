package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Back", group = "Final Auto")
public class PulsarAutoBlueBack extends PulsarAuto {
    @Override
    protected Alliance getAlliance() { return Alliance.BLUE; }

    @Override
    protected StartPosition getStartPosition() { return StartPosition.BACK; }
}
