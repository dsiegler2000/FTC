package org.firstinspires.ftc.teamcode.oldautos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public abstract class ParallelBotAutoCornerOld extends LinearOpMode {

    private ParallelBotHardware robot = new ParallelBotHardware();

    private boolean isBlue;

    private int[] numTimesPicsSeen = {0, 0, 0}; // Left, right, center

    private VuforiaLocalizer vuforia;
    private VuforiaTrackable relicTemplate;
    
    private BNO055IMU imu;

    private Orientation angles;
    private Acceleration gravity;

    public ParallelBotAutoCornerOld(boolean color){

        isBlue = color;

    }

    @Override
    public void runOpMode(){
        
        try{
            
            robot.init(hardwareMap);
        
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "Ab+0cBX/////AAAAGdHqAMTnx0GLk99ODKi2npU8fZTSoYRz3NVvSFAK0EFk6cVF8RTzBiLbhxPYq7ux9X+ATW+W0EXwTqTJYv7a2DyHhScsxg9fzafjr2Ddgdu75ltwpjE/EtNQWfKrSIQJIAespD3AiYczKRK/nQ9txHF9nE9DYht++su01GmV4Hr1KWSwF5H+ZeCTz3Au8NiSGUEPWv6zGmocyTjg00+TcRzAJdf9AFrrZFe1OeiY59egxotwJi7gnYUSfrqL/Mvc79BdDxUENl8FttSNkGxgjtiwjdZBIao7DjYnI21xvIvde98e2i26BOQAuQbn/4eov3Y6G4or0nJUDUIjAzcA0Y6whdiE5qwfd5wdzy9Bkq3J";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            vuforia = ClassFactory.createVuforiaLocalizer(parameters);
    
            VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate");
            
            BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
            gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            gyroParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            gyroParameters.loggingEnabled = false;
            gyroParameters.loggingTag = "IMU";
            gyroParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    
            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            
            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Ready to run");
            telemetry.update();
            
            // Wait for the game to start
            waitForStart();
            
            robot.setBrakes(true);
            
            robot.partiallyRetractRelicFlipper();
            
            relicTrackables.activate();
            
            countVisibleImages();
            
            robot.bringDownBallKnocker();
    
            sleep(700);
    
            float motorPower = 0f;
    
            telemetry.addData("Red", robot.colorSensor.red());
            telemetry.addData("Blue", robot.colorSensor.blue());
            telemetry.update();
    
            if(isBlue){
    
                if(robot.readingBlue()){
                    
                    robot.knockBackward();
    
                }
    
                else {
                    
                    robot.knockForward();
    
                }
                
            }
    
            else{
    
                if(robot.readingBlue()){
                    
                    robot.knockForward();
    
                }
    
                else {
                    
                    robot.knockBackward();
    
                }
    
            }
            
            sleep(400);
            robot.bringUpBallKnocker();
            robot.centerBallKnocker();
            sleep(300);
            
            robot.bringUpBallKnocker();
    
            moveForwardUsingEncoder((isBlue ? -1 : 1) * 150, 500, (isBlue ? -1f : 1f) * 0.3f);
            
            countVisibleImages();
            
            moveForwardUsingEncoder((isBlue ? -1 : 1) * 200, 500, (isBlue ? -1f : 1f) * 0.3f);
            
            moveForwardUsingEncoder((isBlue ? -1 : 1) * 520, 1000, (isBlue ? -1f : 1f) * 0.3f);
            
            countVisibleImages();
            
            int visibleImage = calculateVisibleImage();
            
            if(visibleImage == 0){
                
                telemetry.addData("Detected", "left");
                
            }
            
            else if(visibleImage == 1){
                
                telemetry.addData("Detected", "right");
                
            }
            
            else if(visibleImage == 2){
                
                telemetry.addData("Detected", "center");
                
            }
            
            telemetry.addData("visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
            telemetry.update();
            
            robot.setPower(0f);
            
            // If its blue, do a 180 turn
            if(isBlue){
                
                imu.initialize(gyroParameters);
            
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                float startingAngle = angles.firstAngle;
                
                float turnPower = 0.13f;
                
                robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                robot.frontRight.setPower(-turnPower);
                robot.frontLeft.setPower(turnPower);
                robot.backRight.setPower(-turnPower);
                robot.backLeft.setPower(turnPower);
                
                ElapsedTime time = new ElapsedTime();
                time.reset();
                
                while(Math.abs(startingAngle - angles.firstAngle) < 176f && time.milliseconds() < 6500 && opModeIsActive()){
                    
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
                    telemetry.addData("Starting angle", startingAngle);
                    telemetry.addData("Current angle", angles.firstAngle);
                    telemetry.addData("The numbo", Math.abs(startingAngle - angles.firstAngle));
                    telemetry.addData("time", time.milliseconds());
                    telemetry.update();
                    
                }
                
                robot.setPower(0f);
                
            }
            
            robot.bringUpBallKnocker();
            
            float strafePower = -0.3f;
            int encoderTicks = 0;
            
            if(visibleImage == 1){ // right
                
                encoderTicks = (isBlue ? 900 : 325);
                
            }
            
            if(visibleImage == 0){ // left
                
                encoderTicks = (isBlue ? 260 : 1075);
                
            }
            
            if(visibleImage == 2){ // center
                
                encoderTicks = (isBlue ? 505 : 600);
                
            }
            
            strafePower *= (isBlue ? -1 : 1);
            
            moveUsingEncoder((isBlue ? -1 : 1) * encoderTicks, 2000, -strafePower, strafePower, strafePower, -strafePower);
            
            moveForwardUsingEncoder(700, 1000, 0.3f);
            
            moveForwardUsingEncoder(-400, 1000, -0.1f);
            
            robot.bringUpBallKnocker();
            
     robot.partiallyRetractRelicFlipper();
            robot.flipperMotor.setPower(-0.7f);
            sleep(1700);
            robot.flipperMotor.setPower(1f);
    
            moveForwardUsingEncoder(-400, 1000, -0.5f);
            
     robot.partiallyRetractRelicFlipper();
            robot.flipperMotor.setPower(-0.7f);
            sleep(1320);
            robot.flipperMotor.setPower(0.8f);
            sleep(1000);
            
            moveForwardUsingEncoder(1000, 1000, 0.3f);
            moveForwardUsingEncoder(-100, 1000, -0.3f);
            
            float turnPower = (isBlue ? -1f : 1f) * -0.35f;
            encoderTicks = 130;
            encoderTicks = (!isBlue && visibleImage == 0 ? 30 : encoderTicks);
            encoderTicks = (isBlue && visibleImage == 1 ? 30 : encoderTicks);
            
            moveUsingEncoder((isBlue ? -1 : 1) * encoderTicks, 700, turnPower, -turnPower, turnPower, -turnPower);
            
            robot.bringUpBallKnocker();
            
            robot.setHarvesterPower(0.7f);
            moveForwardUsingEncoder(-1700, 3000, -0.3f);
            robot.bringUpBallKnocker();
            moveForwardUsingEncoder(1800, 3000, 0.3f);
            robot.bringUpBallKnocker();
            
            if(!((visibleImage == 0 && !isBlue) || (visibleImage == 1 && isBlue))){
                
                turnPower *= -1f;
                encoderTicks *= -1f;
        
                moveUsingEncoder((isBlue ? -1 : 1) * encoderTicks, 700, turnPower, -turnPower, turnPower, -turnPower);
                
            }
            

     robot.partiallyRetractRelicFlipper();
            robot.flipperMotor.setPower(-1f);
            sleep(1700);
            robot.flipperMotor.setPower(0.75f);
            robot.setHarvesterPower(0f);
            robot.setPower(0f);
            moveForwardUsingEncoder(-150, 1000, -0.5f);
            moveForwardUsingEncoder(200, 500, 0.4f);
            moveForwardUsingEncoder(-150, 500, -0.3f);

            /*robot.setHarvesterPower(1f);
            moveForwardUsingEncoder(-1000, 1500, -0.5f);
            moveForwardUsingEncoder(-300, 600, -0.2f);
            robot.leftHarvesterMotor.setPower(0f);
            sleep(200);
            robot.setHarvesterPower(0.65f);
            robot.rightHarvesterMotor.setPower(0f);
            sleep(300);
            robot.setHarvesterPower(1f);
            moveForwardUsingEncoder(1300, 1500, 0.5f);
            robot.setHarvesterPower(0f);
            
     robot.partiallyRetractRelicFlipper();
            robot.flipperMotor.setPower(-0.5f);
            sleep(1320);
            robot.flipperMotor.setPower(0.75f);
            
            moveForwardUsingEncoder(-200, 500, -0.35f);
            
            sleep(1000);*/
            
        } catch(InterruptedException e){
            
            
            
        }

    }
    
    private void moveForwardUsingEncoder(int encoderTicks, long millisTimeout, float power) throws InterruptedException{
        
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setTargetPosition(encoderTicks);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        robot.setPower(power);
        
        ElapsedTime time = new ElapsedTime();
        time.reset();     
        
        while(opModeIsActive() && robot.frontRight.isBusy() && time.milliseconds() < millisTimeout){
            
            // telemetry.addData("FR encoder", robot.frontRight.getCurrentPosition());
            // telemetry.update();
            
        }
        
        robot.setPower(0f);
        
    }
    
    private void moveUsingEncoder(int encoderTicks, long millisTimeout, float frPower, float flPower, float brPower, float blPower) throws InterruptedException{
        
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setTargetPosition(encoderTicks);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        robot.frontRight.setPower(frPower);
        robot.frontLeft.setPower(flPower);
        robot.backRight.setPower(brPower);
        robot.backLeft.setPower(blPower);
        
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while(opModeIsActive() && robot.frontRight.isBusy() && time.milliseconds() < millisTimeout) {
            
            telemetry.addData("FR encoder", robot.frontRight.getCurrentPosition());
            telemetry.update();
            
        }
        
        robot.setPower(0f);
        
    }
    
    private void countVisibleImages() throws InterruptedException{

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        numTimesPicsSeen[0] += (vuMark.equals(RelicRecoveryVuMark.LEFT) ? 1 : 0);
        numTimesPicsSeen[1] += (vuMark.equals(RelicRecoveryVuMark.RIGHT) ? 1 : 0);
        numTimesPicsSeen[2] += (vuMark.equals(RelicRecoveryVuMark.CENTER) ? 1 : 0);

    }

    private void resetNumTimesPicSeen() throws InterruptedException{

        numTimesPicsSeen[0] = 0;
        numTimesPicsSeen[1] = 0;
        numTimesPicsSeen[2] = 0;

    }

    private void sleepWhileCountingImages(int millis) throws InterruptedException{

        float startingTime = System.currentTimeMillis();

        while(System.currentTimeMillis() - startingTime < millis){

            countVisibleImages();

        }

    }

    private int calculateVisibleImage() throws InterruptedException{

        float n = (float ) (numTimesPicsSeen[0] + numTimesPicsSeen[1] + numTimesPicsSeen[2]);

        float probabilityLeft = ((float) numTimesPicsSeen[0]) / n;
        float probabilityRight = ((float) numTimesPicsSeen[1]) / n;
        float probabilityCenter = ((float) numTimesPicsSeen[2]) / n;

        if(probabilityLeft > probabilityRight && probabilityLeft > probabilityCenter){

            return 0;

        }

        else if(probabilityRight > probabilityLeft && probabilityRight > probabilityCenter){

            return 1;

        }

        else {

            return 2;

        }

    }
    
    public void breakingSleep(int millis) throws InterruptedException{
        
        if(opModeIsActive()){
            
            sleep(millis);
            
        }
        
    }

}
