package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Flex Sensor Test", group="Tests")
public class FlexSensorTest extends OpMode {
    
    AnalogInput flex;

    @Override
    public void init () {
        
        flex = hardwareMap.analogInput.get("flex");
        
    }

    @Override
    public void loop(){
        
        telemetry.addData("flex", flex.getVoltage());
        telemetry.update();

    }
    
}