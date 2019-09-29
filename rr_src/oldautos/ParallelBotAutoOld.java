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

public abstract class ParallelBotAutoOld extends LinearOpMode {

    private ParallelBotHardware robot = new ParallelBotHardware();

    private boolean isBlue;

    private int[] numTimesPicsSeen = {0, 0, 0}; // Left, right, center

    private VuforiaLocalizer vuforia;
    private VuforiaTrackable relicTemplate;

    private BNO055IMU imu;

    private Orientation angles;
    private Acceleration gravity;

    public ParallelBotAutoOld(boolean color){

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
            
            robot.partiallyRetractRelicFlipper();
            
            robot.camcorderServo.setPosition(0.35f);
            relicTrackables.activate();
            
            countVisibleImages();
            
            robot.bringDownBallKnocker();
    
            breakingSleep(1200);
    
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
                
                telemetry.addData("r", robot.colorSensor.red());
                telemetry.addData("g", robot.colorSensor.green());
                telemetry.addData("b", robot.colorSensor.blue());
                telemetry.update();
    
                if(robot.readingBlue()){
                    
                    robot.knockForward();
    
                }
    
                else {
                    
                    robot.knockBackward();
    
                }
    
            }
            
            breakingSleep(500);
            robot.bringUpBallKnocker();
            robot.centerBallKnocker();
    
            breakingSleep(200);
    
            imu.initialize(gyroParameters);
            
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            float startingAngle = angles.firstAngle;
    
            if(!isBlue){
    
                robot.camcorderServo.setPosition(0.45f);
            
            }
            
            moveForwardUsingEncoder((isBlue ? -1 : 1) * 200, 300, (isBlue ? -1f : 1f) * 0.35f);
            
            countVisibleImages();
            
            moveForwardUsingEncoder((isBlue ? -1 : 1) * 200, 300, (isBlue ? -1f : 1f) * 0.35f);
            
            countVisibleImages();
            
            robot.setBrakes(true);
            
            moveForwardUsingEncoder((isBlue ? -1 : 1) * (isBlue ? 950 : 990), 3000, (isBlue ? -1f : 1f) * 0.35f);
            
            robot.bringUpBallKnocker();
            
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
            
            // moveUsingEncoder(-640, 1400, -turnPower, turnPower, -turnPower, turnPower);
            
            float turnPower = 0.13f;
            
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            robot.frontRight.setPower(-turnPower);
            robot.frontLeft.setPower(turnPower);
            robot.backRight.setPower(-turnPower);
            robot.backLeft.setPower(turnPower);
            
            ElapsedTime time = new ElapsedTime();
            time.reset();
            
            robot.bringUpBallKnocker();
            
            while(Math.abs(startingAngle - angles.firstAngle) < 83f && time.milliseconds() < 3000 && opModeIsActive()){
                
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
                telemetry.addData("Starting angle", startingAngle);
                telemetry.addData("Current angle", angles.firstAngle);
                telemetry.update();
                
            }
            
            robot.setPower(0f);
            
            float strafePower = 0f;
            int encoderTicks = 0;
            
            if(visibleImage == 1){
                
                strafePower = 0.4f;
                encoderTicks = -320;
                
                if(!isBlue){
                    
                    encoderTicks = -300;
                    
                }
                
            }
            
            if(visibleImage == 0){
                
                strafePower = -0.4f;
                encoderTicks = 280;
                
                if(!isBlue){
                    
                    encoderTicks = 315;
                    
                }
                
            }
            
            robot.bringUpBallKnocker();
            
            moveUsingEncoder(encoderTicks, 1400, -strafePower, strafePower, strafePower, -strafePower);
            
            moveForwardUsingEncoder(700, 1000, 0.35f);
            
            float squaredAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            
            moveForwardUsingEncoder(-120, 1000, -0.3f);
            
     robot.partiallyRetractRelicFlipper();
            robot.flipperMotor.setPower(-0.7f);
            breakingSleep(1100);
            robot.flipperMotor.setPower(1f);
            
            robot.bringUpBallKnocker();
            
            moveForwardUsingEncoder(-180, 2000, -0.5f);
            
      robot.partiallyRetractRelicFlipper();
            robot.flipperMotor.setPower(-0.7f);
            breakingSleep(1000);
            robot.flipperMotor.setPower(1f);
            breakingSleep(200);
            
            moveForwardUsingEncoder(310, 2000, 0.3f);
            
            robot.bringUpBallKnocker();
            
            /*turnPower = 0.3f;
            robot.frontRight.setPower(-turnPower);
            robot.frontLeft.setPower(turnPower);
            robot.backRight.setPower(-turnPower);
            robot.backLeft.setPower(turnPower);
            breakingSleep(200);
            moveForwardUsingEncoder(-100, 300, -0.3f);
            turnPower = -0.3f;
            robot.frontRight.setPower(-turnPower);
            robot.frontLeft.setPower(turnPower);
            robot.backRight.setPower(-turnPower);
            robot.backLeft.setPower(turnPower);
            breakingSleep(200);
            robot.setPower(0f);*/
            
            if(visibleImage == 2){
                
                moveForwardUsingEncoder(-100, 2000, -0.3f);
                
                strafePower = 0.35f;
                moveUsingEncoder(-20, 500, -strafePower, strafePower, strafePower, -strafePower);
                
            }
            
            // turnPower = 0.13f;
            
            if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < squaredAngle){
                
                turnPower *= -1f;
                
            }
            
            robot.bringUpBallKnocker();
            
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            robot.frontRight.setPower(-turnPower);
            robot.frontLeft.setPower(turnPower);
            robot.backRight.setPower(-turnPower);
            robot.backLeft.setPower(turnPower);
            
            time.reset();
            
            while(Math.abs(squaredAngle + 3f - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > 3f && time.milliseconds() < 3000 && opModeIsActive()){
                
                telemetry.addData("visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
                telemetry.addData("target angle", squaredAngle);
                telemetry.addData("Current angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();
                
            }
            
            /*robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turnPower = visibleImage == 0 ? -0.35f : 0.35f;
            robot.frontRight.setPower(turnPower);
            robot.frontLeft.setPower(-turnPower);
            robot.backRight.setPower(turnPower);
            robot.backLeft.setPower(-turnPower);
            breakingSleep(30);
            robot.setPower(0f);*/
        
            // in the pit
            robot.setHarvesterPower(0.7f);
            moveForwardUsingEncoder(-500, 2000, -0.5f);
            moveForwardUsingEncoder(-420, 2000, -0.25f);
            robot.leftHarvesterMotor.setPower(0f);
            breakingSleep(100);
            robot.setHarvesterPower(0.7f);
            moveForwardUsingEncoder(350, 1500, 0.5f);
            
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turnPower = 0.35f;
            breakingSleep(10);
            robot.frontRight.setPower(turnPower);
            robot.frontLeft.setPower(-turnPower);
            robot.backRight.setPower(turnPower);
            robot.backLeft.setPower(-turnPower);
            breakingSleep(50);
            robot.setPower(0f);
            
            moveForwardUsingEncoder(-300, 2000, -0.4f);
            
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            /*turnPower = -0.35f;
            robot.frontRight.setPower(turnPower);
            robot.frontLeft.setPower(-turnPower);
            robot.backRight.setPower(turnPower);
            robot.backLeft.setPower(-turnPower);
            breakingSleep(((visibleImage == 0 || visibleImage == 2) ? 450 : 100));
            robot.setPower(0f);*/
            
            robot.setHarvesterPower(0.65f);
            breakingSleep(100);
            robot.leftHarvesterMotor.setPower(0f);
            breakingSleep(300);
            robot.setHarvesterPower(0.7f);
            
            turnPower = 0.2f;
            
            if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < squaredAngle){
                
                turnPower *= -1f;
                
            }
            
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            robot.frontRight.setPower(-turnPower);
            robot.frontLeft.setPower(turnPower);
            robot.backRight.setPower(-turnPower);
            robot.backLeft.setPower(turnPower);
            
            time.reset();
            
            while(Math.abs(squaredAngle + 3f - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > 3f && time.milliseconds() < 800 && opModeIsActive()){
                
                telemetry.addData("visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
                telemetry.addData("target angle", squaredAngle);
                telemetry.addData("starting angle", squaredAngle);
                telemetry.addData("Current angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();
                
            }
            
            float finalAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            
            turnPower = 0.4f;
            
            robot.setPower(0f);
            
            if(visibleImage == 1){
                
                turnPower = 0.3f * (isBlue ? -1f : 1f);
                robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                robot.frontRight.setPower(-turnPower);
                robot.frontLeft.setPower(turnPower);
                robot.backRight.setPower(-turnPower);
                robot.backLeft.setPower(turnPower); 
                
                breakingSleep(50);
                robot.setPower(0f);
                
            }
            
            else if(visibleImage == 0){
                
                turnPower = -0.3f * (isBlue ? -1f : 1f);;
                robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                robot.frontRight.setPower(-turnPower);
                robot.frontLeft.setPower(turnPower);
                robot.backRight.setPower(-turnPower);
                robot.backLeft.setPower(turnPower); 
                
                breakingSleep(50);
                robot.setPower(0f);
                
            }
            
            moveForwardUsingEncoder(1000, 2000, 0.5f);
            moveForwardUsingEncoder(500, 1000, 0.4f);
            robot.leftHarvesterMotor.setPower(0f);
            breakingSleep(200);
            robot.setHarvesterPower(0.7f);
            robot.rightHarvesterMotor.setPower(0f);
            breakingSleep(200);
            robot.setHarvesterPower(0.7f);
            breakingSleep(100);
            
            robot.liftMotor.setPower(-0.45f);
            moveForwardUsingEncoder(-60, 500, -0.6f);
            // sleep(10);
            robot.setHarvesterPower(0f);
            // sleep(35);
            robot.liftMotor.setPower(0f);
    
            robot.partiallyRetractRelicFlipper();
            robot.flipperMotor.setPower(-1f);
            breakingSleep(1700);
            
            moveForwardUsingEncoder(-170, 2000, -0.6f);
            moveForwardUsingEncoder(200, 2000, 0.4f);
            moveForwardUsingEncoder(-50, 400, -0.3f);
            robot.flipperMotor.setPower(1f);
            robot.setPower(0f);
            robot.setHarvesterPower(0f);
            
            robot.bringDownBallKnocker();
            robot.setHarvesterPower(0f);
    
            breakingSleep(700);
            
        }catch(InterruptedException e){
            
            
            
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
        
        while(opModeIsActive() && robot.frontRight.isBusy() && time.milliseconds() < millisTimeout) {
            
            telemetry.addData("FR encoder", robot.frontRight.getCurrentPosition());
            telemetry.update();
            
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

        while(opModeIsActive() && robot.frontRight.isBusy() && time.milliseconds() < millisTimeout){
            
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
