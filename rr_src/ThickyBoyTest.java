package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ThickyBoyTest", group="ThickyBoy")
public class ThickyBoyTest extends OpMode {
    
    public CRServo extender1;

    public Servo gripper;
    public Servo flipper;
    
    @Override
    public void init(){
        
        extender1 = hardwareMap.crservo.get("extender1");
        extender1.setPower(0);
        
        flipper = hardwareMap.servo.get("flipper");
        flipper.setPosition(0);
        
        gripper = hardwareMap.servo.get("gripper");
        gripper.setPosition(0);

    } 
    
    @Override
    public void loop(){
        
        extender1.setPower(Math.max(-0.8f, Math.min(0.8f, gamepad2.right_trigger - gamepad2.left_trigger)));

        flipper.setPosition(flipper.getPosition() + (gamepad1.y ? 0.01 : 0));
        flipper.setPosition(flipper.getPosition() - (gamepad1.a ? 0.01 : 0));
        
        gripper.setPosition(gripper.getPosition() + (gamepad2.y ? 0.01 : 0));
        gripper.setPosition(gripper.getPosition() - (gamepad2.a ? 0.01 : 0));
        
        telemetry.addData("extender1 power", extender1.getPower());
        telemetry.addData("gripper pos", gripper.getPosition());
        telemetry.addData("flipper pos", flipper.getPosition());
        telemetry.update();
        
    }

}