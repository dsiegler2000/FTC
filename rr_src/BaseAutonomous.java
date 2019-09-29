package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class BaseAutonomous extends LinearOpMode {

    protected ParallelBotHardware robot;
    // BaseAuto extends LinearOpMode, ParallelBotAuto extends this, ParallelBotAutoRed EXTENDS LINEAROPMODE

    protected boolean isBlue;

    protected int[] numTimesPicsSeen = {0, 0, 0}; // Left, right, center

    protected VuforiaLocalizer vuforia;
    protected VuforiaTrackable relicTemplate;

    protected BNO055IMU imu;

    protected Orientation angles;
    protected Acceleration gravity;

    protected VuforiaTrackables relicTrackables;

    protected BNO055IMU.Parameters gyroParameters;
    
    protected DcMotorEx rightHarvesterMotorEx;
    protected DcMotorEx leftHarvesterMotorEx;

    public BaseAutonomous(boolean isBlue){

        this.isBlue = isBlue;
        
        robot = new ParallelBotHardware();

    }

    @Override
    public void runOpMode() {

        // Init the robot
        robot.init(hardwareMap);

        rightHarvesterMotorEx = (DcMotorEx) robot.rightHarvesterMotor;
        leftHarvesterMotorEx = (DcMotorEx) robot.leftHarvesterMotor;
        
        robot.retractFlexSensor();

        // Set up Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ab+0cBX/////AAAAGdHqAMTnx0GLk99ODKi2npU8fZTSoYRz3NVvSFAK0EFk6cVF8RTzBiLbhxPYq7ux9X+ATW+W0EXwTqTJYv7a2DyHhScsxg9fzafjr2Ddgdu75ltwpjE/EtNQWfKrSIQJIAespD3AiYczKRK/nQ9txHF9nE9DYht++su01GmV4Hr1KWSwF5H+ZeCTz3Au8NiSGUEPWv6zGmocyTjg00+TcRzAJdf9AFrrZFe1OeiY59egxotwJi7gnYUSfrqL/Mvc79BdDxUENl8FttSNkGxgjtiwjdZBIao7DjYnI21xvIvde98e2i26BOQAuQbn/4eov3Y6G4or0nJUDUIjAzcA0Y6whdiE5qwfd5wdzy9Bkq3J";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        // Set up the gyro
        gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // See the calibration sample opmode
        gyroParameters.loggingEnabled = false;
        gyroParameters.loggingTag = "IMU";
        gyroParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Get the IMU (must be done here)
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        relicTrackables.activate();

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

    }
    
    protected void gyroTurn(float turnPower, float startingAngle, float targetAngle, int millisTimeout){
        
        gyroTurn(turnPower, startingAngle, targetAngle, millisTimeout, false);
        
    }

    protected void gyroTurn(float turnPower, float startingAngle, float targetAngle, int millisTimeout, boolean usePropertional){

        // Reset all of the encoders
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.frontRight.setPower(-turnPower);
        robot.frontLeft.setPower(turnPower);
        robot.backRight.setPower(-turnPower);
        robot.backLeft.setPower(turnPower);
        
        boolean whichWay = Math.abs(startingAngle) < targetAngle;
        
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        telemetry.addData("which way", whichWay);
        telemetry.addData("curr", angles.firstAngle);
        telemetry.addData("tar", targetAngle);
        telemetry.update();
        
        float maxSpeed = 0.6f;
        float maxErr = Math.abs(targetAngle - angles.firstAngle);

        ElapsedTime time = new ElapsedTime();
        time.reset();

        // Bring up ball knocker if it went limp
        robot.bringUpBallKnocker();

        // Turn until timeout or angle reached
        while((whichWay ? Math.abs(angles.firstAngle) < targetAngle : Math.abs(angles.firstAngle) > Math.abs(targetAngle)) && time.milliseconds() < millisTimeout && opModeIsActive()){

            float err = Math.abs(Math.abs(angles.firstAngle) - targetAngle);
            float percentageDoneWithTurn = (err / maxErr);
            float range = maxSpeed - turnPower;
            float s = turnPower + range * percentageDoneWithTurn;
            s = (usePropertional ? s : turnPower);
            
            robot.frontRight.setPower(-s);
            robot.frontLeft.setPower(s);
            robot.backRight.setPower(-s);
            robot.backLeft.setPower(s);  

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("turn power", s);
            telemetry.addData("err", err);
            telemetry.addData("max err", maxErr);
            telemetry.addData("% done", percentageDoneWithTurn);
            telemetry.addData("range", range);
            telemetry.addData("Visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
            telemetry.addData("Starting angle", startingAngle);
            telemetry.addData("Current angle", angles.firstAngle);
            telemetry.update();

        }

        robot.setPower(0f);
        
    }

    protected void gyroSquareUp(float turnPower, float squaredAngle, int timeoutMillis){

        // In case limp
        robot.bringUpBallKnocker();

        if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < squaredAngle){

            turnPower *= -1f;

        }

        // Reset encoders
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.frontRight.setPower(-turnPower);
        robot.frontLeft.setPower(turnPower);
        robot.backRight.setPower(-turnPower);
        robot.backLeft.setPower(turnPower);

        ElapsedTime time = new ElapsedTime();
        time.reset();

        // Turn until almost square
        while(Math.abs(squaredAngle + 3f - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > 3f && time.milliseconds() < timeoutMillis && opModeIsActive()){

            telemetry.addData("Visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
            telemetry.addData("Target angle", squaredAngle);
            telemetry.addData("Current angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();

        }

        robot.setPower(0f);

    }

    protected void moveForwardUsingEncoder(int encoderTicks, long millisTimeout, float power) throws InterruptedException {

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setTargetPosition(encoderTicks);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setPower(power);

        ElapsedTime time = new ElapsedTime();
        time.reset();
        

        while(opModeIsActive() && robot.frontRight.isBusy() && time.milliseconds() < millisTimeout) {

            telemetry.addData("FR encoder", robot.frontRight.getCurrentPosition());
            telemetry.addData("Visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
            telemetry.update();

        }

        robot.setPower(0f);
    }

    protected void moveUsingEncoder(int encoderTicks, long millisTimeout, float frPower, float flPower, float brPower, float blPower) throws InterruptedException {

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
            telemetry.addData("Visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
            telemetry.update();

        }

        robot.setPower(0f);

    }

    protected void countVisibleImages() throws InterruptedException {

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        numTimesPicsSeen[0] += (vuMark.equals(RelicRecoveryVuMark.LEFT) ? 1 : 0);
        numTimesPicsSeen[1] += (vuMark.equals(RelicRecoveryVuMark.RIGHT) ? 1 : 0);
        numTimesPicsSeen[2] += (vuMark.equals(RelicRecoveryVuMark.CENTER) ? 1 : 0);

    }

    protected int calculateVisibleImage() throws InterruptedException {

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

    protected void breakingSleep(int millis) throws InterruptedException {

        if(opModeIsActive()){

            sleep(millis);

        }

    }

    protected void knockJewel() throws InterruptedException {

        robot.bringUpBallKnocker();
        sleep(200);
        
        robot.centerBallKnocker();
        robot.bringDownBallKnocker();

        breakingSleep(1000);

        telemetry.addData("r, g, b", robot.colorSensor.red() + ", " + robot.colorSensor.green() + ", " + robot.colorSensor.blue());
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

        breakingSleep(400);
        robot.bringUpBallKnocker();
        robot.centerBallKnocker();
        breakingSleep(300);

    }
    
    public void driveStraight(float targetAngle, float targetSpeed, int encoderTicks, int millisTimeout) throws InterruptedException {

        driveStraight(targetAngle, targetSpeed, encoderTicks, millisTimeout, 75);

    }

    public void driveStraight(float targetAngle, float targetSpeed, int encoderTicks, int millisTimeout, float kP) throws InterruptedException {

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.setPower(targetSpeed);

        ElapsedTime time = new ElapsedTime();
        time.reset();

        while(opModeIsActive() && (encoderTicks < 0 ? (robot.frontRight.getCurrentPosition() > encoderTicks) : (robot.frontRight.getCurrentPosition() < encoderTicks)) && time.milliseconds() < millisTimeout) {

            float diff;
            float gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            if (targetAngle == 0) {

                if(gyroHeading > 180) {

                    diff = gyroHeading - 360;

                }

                else {

                    diff = gyroHeading - 0;

                }

            }

            else {

                diff = gyroHeading - targetAngle;

            }

            telemetry.addData("Gyro: ", gyroHeading);
            float leftDrivePower = targetSpeed + diff / kP;
            float rightDrivePower = targetSpeed - diff / kP;

            if(leftDrivePower > 1.0){

                leftDrivePower = 1.0f;

            }

            if(leftDrivePower < -1.0) {

                leftDrivePower = -1.0f;

            }

            if(rightDrivePower > 1.0) {

                rightDrivePower = 1.0f;

            }

            if(rightDrivePower < -1.0) {

                rightDrivePower = -1.0f;

            }

            robot.setPower(rightDrivePower, leftDrivePower, rightDrivePower, leftDrivePower);

            telemetry.addData("FR encoder", robot.frontRight.getCurrentPosition());
            telemetry.addData("Visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
            telemetry.update();

        }

        robot.setPower(0f);

    }

}
