package edu.usrobotics.opmode.mecanumbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by mborsch19 & dsiegler19 on 10/13/16.
 */
public abstract class MecanumAuto extends LinearOpMode {

    private MecanumBotHardware robot = new MecanumBotHardware();

    private boolean isBlue;

    private VuforiaLocalizer vuforia;
    private VuforiaTrackable relicTemplate;

    private int[] numTimesPicsSeen = {0, 0, 0}; // Left, right, center

    public MecanumAuto(boolean color){

        isBlue = color;

    }

    @Override
    public void runOpMode(){

        robot.init(hardwareMap);
        
        robot.closeGripper();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ab+0cBX/////AAAAGdHqAMTnx0GLk99ODKi2npU8fZTSoYRz3NVvSFAK0EFk6cVF8RTzBiLbhxPYq7ux9X+ATW+W0EXwTqTJYv7a2DyHhScsxg9fzafjr2Ddgdu75ltwpjE/EtNQWfKrSIQJIAespD3AiYczKRK/nQ9txHF9nE9DYht++su01GmV4Hr1KWSwF5H+ZeCTz3Au8NiSGUEPWv6zGmocyTjg00+TcRzAJdf9AFrrZFe1OeiY59egxotwJi7gnYUSfrqL/Mvc79BdDxUENl8FttSNkGxgjtiwjdZBIao7DjYnI21xvIvde98e2i26BOQAuQbn/4eov3Y6G4or0nJUDUIjAzcA0Y6whdiE5qwfd5wdzy9Bkq3J";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start
        waitForStart();
        
        // robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 10);

        telemetry.addData("Pos:", "1");
        telemetry.update();

        relicTrackables.activate();
        
        telemetry.addData("Pos:", "2");
        telemetry.update();

        countVisibleImages();
        
        telemetry.addData("Pos:", "3");
        telemetry.update();

        robot.bringDownBallKnocker();

        sleep(500);
        // set mode run to position
        // set target position
        // set power
        // sleepWhileCountingImages(500);

        float motorPower = 0f;

        telemetry.addData("Red", robot.colorSensor.red());
        telemetry.addData("Blue", robot.colorSensor.blue());
        telemetry.update();

        if(isBlue){

            if(robot.readingBlue()){
                
                robot.knockForward();

            }

            else {
                
                robot.knockBackward();

            }
            
            sleep(150);
            robot.centerBallKnocker();
            
        }

        else{

            if(robot.readingBlue()){
                
                robot.knockBackward();

            }

            else {
                
                robot.knockForward();

            }

        }
        
        sleep(150);
        robot.centerBallKnocker();

        sleep(150);
        // sleepWhileCountingImages(200);
        robot.bringUpBallKnocker();
        robot.centerBallKnocker();
        // sleepWhileCountingImages(100);
        sleep(300);

        int visibleImage = calculateVisibleImage();
        visibleImage = 3;
        resetNumTimesPicSeen();

        telemetry.addData("Visible image (1=left, 2=r, 3=c)", visibleImage);
        telemetry.update();

        int time = 0;

        switch (visibleImage){

            case 0:
                time = 1000;
                break;

            case 1:
                time = 1500;
                break;

            default:
                time = 2000;
                break;

        }

        motorPower = 0.5f;
        robot.frontLeft.setPower(motorPower - 0.1f);
        robot.backLeft.setPower(motorPower - 0.1f);
        robot.frontRight.setPower(motorPower + 0.1f);
        robot.backRight.setPower(motorPower + 0.1f);
        sleep(time);
        robot.setPower(0);

        // Orientation headings = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // float startingAngle = headings.firstAngle;

        float turnPower = 0.4f;

        robot.frontLeft.setPower(turnPower);
        robot.backLeft.setPower(turnPower);
        robot.frontRight.setPower(-turnPower);
        robot.backRight.setPower(-turnPower);

        // while(Math.abs(robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startingAngle) < 90){}

        sleep(1600);

        robot.setPower(0);

        sleep(100);
        robot.setPower(0.5f);
        sleep(320);
        robot.openGripper();
        robot.setPower(0.5f);
        sleep(500);
        robot.setPower(-0.5f);
        sleep(400);
        robot.setPower(0.5f);
        sleep(500);
        robot.setPower(-0.5f);
        sleep(400);
        robot.setPower(0f);

    }

    private void countVisibleImages(){

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        numTimesPicsSeen[0] += (vuMark.equals(RelicRecoveryVuMark.LEFT) ? 1 : 0);
        numTimesPicsSeen[1] += (vuMark.equals(RelicRecoveryVuMark.RIGHT) ? 1 : 0);
        numTimesPicsSeen[2] += (vuMark.equals(RelicRecoveryVuMark.CENTER) ? 1 : 0);

    }

    private void resetNumTimesPicSeen(){

        numTimesPicsSeen[0] = 0;
        numTimesPicsSeen[1] = 0;
        numTimesPicsSeen[2] = 0;

    }

    private void sleepWhileCountingImages(int millis){

        float startingTime = System.currentTimeMillis();

        while(System.currentTimeMillis() - startingTime < millis){

            countVisibleImages();

        }

    }

    private int calculateVisibleImage(){

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

}