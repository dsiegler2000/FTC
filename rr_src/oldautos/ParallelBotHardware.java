package org.firstinspires.ftc.teamcode.oldautos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;


public class ParallelBotHardware {
    
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;

    public boolean frCorrectDirection = false;
    public boolean flCorrectDirection = true;
    public boolean brCorrectDirection = false;
    public boolean blCorrectDirection = true;
    
    public DcMotor liftMotor;
    public DcMotor flipperMotor;
    
    public DcMotor leftHarvesterMotor;
    public DcMotor rightHarvesterMotor;
    
    public Servo ballKnocker;
    public Servo ballKnockerPivoter;
    
    public Servo stonerLeft;
    public Servo stonerRight;
    
    public Servo camcorderServo;
    
    public Servo parkingBrake;
    
    public ColorSensor colorSensor;

    public DigitalChannel limitSwitch;
    
    public CRServo extender;
    public Servo relicFlipper;
    public Servo relicGripper;
    
    public OpticalDistanceSensor testDistance;
    
    public void init(HardwareMap hm){
        
        initHardware(hm);
        
    }
    
    private void initHardware(HardwareMap hardwareMap){
        
        testDistance = hardwareMap.opticalDistanceSensor.get("testdist");
        
        liftMotor = hardwareMap.dcMotor.get("lm");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flipperMotor = hardwareMap.dcMotor.get("fm");
        flipperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flipperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftHarvesterMotor = hardwareMap.dcMotor.get("lh");
        leftHarvesterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHarvesterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        rightHarvesterMotor = hardwareMap.dcMotor.get("rh");
        rightHarvesterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHarvesterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        leftHarvesterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightHarvesterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        frontRight = hardwareMap.dcMotor.get ("fr");
        frontLeft = hardwareMap.dcMotor.get ("fl");
        backRight = hardwareMap.dcMotor.get ("br");
        backLeft = hardwareMap.dcMotor.get ("bl");
        
        extender = hardwareMap.crservo.get("extender");
        relicFlipper = hardwareMap.servo.get("rf");
        relicGripper = hardwareMap.servo.get("rg");
        
        stonerLeft = hardwareMap.servo.get("sl");
        stonerRight = hardwareMap.servo.get("sr");
        
        parkingBrake = hardwareMap.servo.get("ps");
        
        colorSensor = hardwareMap.colorSensor.get("cs");
        colorSensor.enableLed(false);

        limitSwitch = hardwareMap.digitalChannel.get("ls");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        frontRight.setDirection(frCorrectDirection ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(flCorrectDirection ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(brCorrectDirection ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(blCorrectDirection ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);    
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER); 
        
        ballKnocker = hardwareMap.servo.get("bk");
        ballKnockerPivoter = hardwareMap.servo.get("bkp");
        
        camcorderServo = hardwareMap.servo.get("camservo");
        retractCamcorder();

        initBallKnocker();
        centerBallKnocker();
        
        closeRelicGripper();
        retractRelicFlipper();
        
        harvesterDown();

    }
    
    public void harvesterDown(){
        
        stonerRight.setPosition(1f);
        stonerLeft.setPosition(0f);
        
    }
    
    public void harvesterUp(){
        
        stonerRight.setPosition(0.5f);
        stonerLeft.setPosition(0.45f);
        
    }
    
    public void setPower(float power){
        
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
        
    }
    
    public float openRelicGripper(){
        
        relicGripper.setPosition(0.45f);
        
        return 0.45f;
        
    }
    
    public float closeRelicGripper(){
        
        relicGripper.setPosition(0.78f);
        
        return 0.78f;
        
    }
    
    public float extendRelicFlipper(){
        
        relicFlipper.setPosition(0.8f);
        
        return 0.8f;
        
    }
    
    public float retractRelicFlipper(){
        
        relicFlipper.setPosition(0.03f);
        
        return 0.03f;
        
    }
    
    public float partiallyRetractRelicFlipper(){
        
        relicFlipper.setPosition(0.45f);
        
        return 0.45f;
        
    }
    
    public void initBallKnocker(){
        
        ballKnocker.setPosition(0.75f);
        
    }
    
    public void bringUpBallKnocker(){

        ballKnocker.setPosition(0.6f);

    }
    
    public void centerBallKnocker(){
        
        ballKnockerPivoter.setPosition(0.148f);
        
    }
    
    public void bringDownBallKnocker(){
    
        ballKnocker.setPosition(0.3f);   
        
    }
    
    public void knockForward(){
        
        ballKnockerPivoter.setPosition(0f);
        
    }
    
    public void knockBackward(){
        
        ballKnockerPivoter.setPosition(0.265f);
        
    }
    
    public boolean readingRed(){

        return colorSensor.red() > colorSensor.blue();

    }

    public boolean readingBlue(){

        return colorSensor.blue() > colorSensor.red();

    }
    
    public void setHarvesterPower(float power){
        
        leftHarvesterMotor.setPower(power);
        rightHarvesterMotor.setPower(power);
        
    }
    
    public void retractCamcorder(){
        
        camcorderServo.setPosition(0.365f);
        
    }
    
    public void extendCamcorder(){
        
        camcorderServo.setPosition(0.616f);
        
    }

    public void setBrakes(boolean active){

        frontRight.setZeroPowerBehavior(active ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(active ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(active ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(active ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void setPower(float fr, float fl, float br, float bl){

        frontRight.setPower(fr);
        frontLeft.setPower(fl);
        backRight.setPower(br);
        backLeft.setPower(bl);

    }
    
}
