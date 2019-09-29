package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Grippy flippy boi test", group="Tests")
public class GrippyFlippyTest extends OpMode {
    
    Servo grip1;
    Servo grip2;

    @Override
    public void init () {
        
        grip1 = hardwareMap.servo.get("g1");
        grip2 = hardwareMap.servo.get("g2");
        
    }

    @Override
    public void loop(){
        
        if(gamepad1.a){
            
            grip1.setPosition(grip1.getPosition() + 0.01f);
            
        }
        
        if(gamepad1.b){
            
            grip1.setPosition(grip1.getPosition() - 0.01f);
            
        }
        
        if(gamepad1.x){
            
            grip2.setPosition(grip2.getPosition() + 0.01f);
            
        }
        
        if(gamepad1.y){
            
            grip2.setPosition(grip2.getPosition() - 0.01f);
            
        }
        
        telemetry.addData("g1", grip1.getPosition());
        telemetry.addData("g2", grip2.getPosition());
        telemetry.update();

    }
    
}