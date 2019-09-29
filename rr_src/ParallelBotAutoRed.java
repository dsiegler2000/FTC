package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="ParallelBot Red", group="ParallelBot")
public class ParallelBotAutoRed extends ParallelBotAuto {

    public ParallelBotAutoRed(){

        super(false);
        
        telemetry.addLine("ooof");
        telemetry.update();

    }

}
