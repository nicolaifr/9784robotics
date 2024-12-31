package hardware.commands.intothedeep.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import hardware.HardwareBase;

public class autoCommands extends HardwareBase {
    //all arm and claw commands for auto go here
    //NICOLAI DO THIS
    //do FSM for now


    // switch (state)
        //case 0:
        //if (!inProgress)
        //timer.reset
        //state++
        //case 1:
        //if timer.time.milliseconds >= however long
            //timer.reset()
            //state++
    boolean inProgress;
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
