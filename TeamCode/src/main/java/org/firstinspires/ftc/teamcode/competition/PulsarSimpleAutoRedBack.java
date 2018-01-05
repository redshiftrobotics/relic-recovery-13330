package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Back (Simple)", group = "Final Auto Simple")
public class PulsarSimpleAutoRedBack extends PulsarAuto {
    @Override
    protected Alliance getAlliance() { return Alliance.RED; }

    @Override
    protected StartPosition getStartPosition() { return StartPosition.BACK; }

    @Override
    protected boolean isSimpleAuto() { return true; }
}
