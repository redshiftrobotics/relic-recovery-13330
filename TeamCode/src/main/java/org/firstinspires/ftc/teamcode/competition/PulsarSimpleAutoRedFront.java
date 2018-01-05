package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Front (Simple)", group = "Final Auto Simple")
public class PulsarSimpleAutoRedFront extends PulsarAuto {
    @Override
    protected Alliance getAlliance() { return Alliance.RED; }

    @Override
    protected StartPosition getStartPosition() { return StartPosition.FRONT; }

    @Override
    protected boolean isSimpleAuto() { return true; }
}
