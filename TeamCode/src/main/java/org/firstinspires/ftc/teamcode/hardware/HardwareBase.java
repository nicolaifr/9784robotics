package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HardwareBase
{

    HardwareMap hwMap;

    Telemetry telemetry = null;

    String hardwareName;

    /* Constructor */
    public HardwareBase(){
        hardwareName = "Unknown";
    }

    public HardwareBase(String n){
        hardwareName = n;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry t) {
        hwMap = ahwMap;
        telemetry = t;
    }

    public void start() {}

    public void reset () {}

    public void stop() {}

    public void setTelemetry (Telemetry t)
    {
        telemetry = t;
    }


}