package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Front (Simple)", group = "Final Auto Simple")
public class PulsarSimpleAutoBlueFront extends PulsarAuto {
    @Override
    protected Alliance getAlliance() { return Alliance.BLUE; }

    @Override
    protected StartPosition getStartPosition() { return StartPosition.FRONT; }

    @Override
    protected boolean isSimpleAuto() { return true; }
}
