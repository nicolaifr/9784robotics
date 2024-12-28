package org.firstinspires.ftc.teamcode.hardware.commands.intothedeep.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HardwareBase;

public class autoCommands extends HardwareBase {
    //all arm and claw commands for auto go here
    //NICOLAI DO THIS
    //do FSM for now
    clawAutoCommands claw;
    slidesAutoCommands slides;
    @Override
    public void init(HardwareMap ahwMap, Telemetry t) {
        super.init(ahwMap, t);
        claw = new clawAutoCommands();
        slides = new slidesAutoCommands();

        claw.init(ahwMap,t);
        slides.init(ahwMap, t);
    }
}
