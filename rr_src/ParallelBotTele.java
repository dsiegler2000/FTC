package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name="ParallelBot TeleOp", group="ParallelBot")
public class ParallelBotTele extends OpMode {

    private ParallelBotHardware robot = new ParallelBotHardware();

    private boolean controlsReversed = false;
    private boolean relicMode = false;
    private boolean superSlowMode = false;

    private float relicFlipperPosition;
    private float relicGrabberPosition;

    private boolean bButtonPressedLastTime = false;
    private boolean yButton2PressedLastTime = false;
    private boolean yButton1PressedLastTime = false;

    private boolean braking = false;
    private boolean rightBumperPressedLastTime = false;

    private boolean resettingFlipper = false;
    private ElapsedTime resetTimer = new ElapsedTime();

    private ElapsedTime autoFlipperUpTimer = new ElapsedTime();
    private boolean autoFlipperUp = false;
    private int autoParkingForwardMillis = 700;

    private DcMotorEx flipperMotorEx;
    private DcMotorEx liftMotorEx;

    private DcMotorEx rightHarvesterMotorEx;
    private DcMotorEx leftHarvesterMotorEx;

    @Override
    public void init () {

        robot.init(hardwareMap, false);

        flipperMotorEx = (DcMotorEx) robot.flipperMotor;
        liftMotorEx = (DcMotorEx) robot.liftMotor;

        rightHarvesterMotorEx = (DcMotorEx) robot.rightHarvesterMotor;
        leftHarvesterMotorEx = (DcMotorEx) robot.leftHarvesterMotor;

        robot.rightHarvesterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftHarvesterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void start(){
        
        robot.bringUpBallKnocker();

        robot.initMovements();

        relicFlipperPosition = robot.partiallyRetractRelicFlipper(relicMode);
        relicGrabberPosition = robot.openRelicGripper();

        robot.parkingBrake.setPosition(0.189f);

    }

    @Override
    public void loop(){

        if(gamepad1.dpad_right){

            robot.parkingBrake.setPosition(0.189f);

        }

        if(gamepad1.dpad_left){

            robot.parkingBrake.setPosition(0.628f);

        }

        telemetry.addData("the memer", robot.glyphSensor.getLightDetected());

        telemetry.addData("parking break pos", robot.parkingBrake.getPosition());

        // Bring the harvester up or down for parking
        if(gamepad1.dpad_up){

            robot.harvesterUp();

        }

        if(gamepad1.dpad_down){

            robot.harvesterDown();
        }

        // Automatic flipper reset
        if(gamepad2.left_stick_button && !resettingFlipper){

            resettingFlipper = true;
            resetTimer.reset();
            robot.flipperMotor.setPower(0.8f);

        }

        if(resetTimer.milliseconds() > 1000){

            resettingFlipper = false;

        }

        float rightStickY = gamepad2.right_stick_y;
        float leftStickY = gamepad2.left_stick_y;

        float liftPower = leftStickY * 0.5f;
        float flipperPower = rightStickY * (gamepad2.right_stick_button ? 1f : 0.5f);

        boolean manualMode = false;

        if(resettingFlipper && (Math.abs(rightStickY) > 0.1f || Math.abs(leftStickY) > 0.1f) && resetTimer.milliseconds() > 300){

            manualMode = true;
            resettingFlipper = false;

        }

        else if(!resettingFlipper){

            manualMode = true;

        }

        if(gamepad1.x){

            autoFlipperUp = true;
            autoFlipperUpTimer.reset();
            robot.setBrakes(true);

        }

        if(manualMode && !relicMode){

            if(!autoFlipperUp){

                robot.flipperMotor.setPower(flipperPower);

            }

            if(!robot.limitSwitch.getState() && liftPower > 0f){

                liftPower = 0f;

            }

            robot.liftMotor.setPower(liftPower);

        }

        // Detect if jamming
        double leftHarvesterSpeed = leftHarvesterMotorEx.getVelocity(AngleUnit.DEGREES);
        double rightHarvesterSpeed = rightHarvesterMotorEx.getVelocity(AngleUnit.DEGREES);

        boolean jammed = leftHarvesterSpeed < 50d || rightHarvesterSpeed < 50d;

        telemetry.addData("left", leftHarvesterSpeed);
        telemetry.addData("right", rightHarvesterSpeed);
        telemetry.addData("jammed", jammed);

        robot.rightHarvesterMotor.setPower((gamepad2.right_bumper ? -1f : 1f) * gamepad2.right_trigger * 0.8f);
        robot.leftHarvesterMotor.setPower((gamepad2.left_bumper ? -1f : 1f) * gamepad2.left_trigger * 0.8f);

        // rightHarvesterMotorEx.setVelocity(10000, AngleUnit.DEGREES);
        // 870

        if(relicMode){

            if(gamepad2.x){

                relicGrabberPosition = robot.openRelicGripper();

            }

            if(gamepad2.a){

                relicGrabberPosition = robot.closeRelicGripper();

            }

            if(gamepad2.left_bumper){

                relicFlipperPosition = robot.partiallyRetractRelicFlipper(relicMode);

            }

            if(gamepad2.right_bumper){

                relicFlipperPosition = robot.extendRelicFlipper();

            }

            if(gamepad2.dpad_right){

                robot.relicGripper.setPosition(robot.relicGripper.getPosition() + 0.01f);
                relicGrabberPosition = (float) (robot.relicGripper.getPosition() + 0.01f);

            }

            if(gamepad2.dpad_left){

                robot.relicGripper.setPosition(robot.relicGripper.getPosition() - 0.01f);
                relicGrabberPosition = (float) (robot.relicGripper.getPosition() - 0.01f);

            }

            robot.relicFlipper.setPosition(robot.relicFlipper.getPosition() + 0.007f * rightStickY);
            relicFlipperPosition = (float) (robot.relicFlipper.getPosition() + 0.007f * rightStickY);

            robot.extender.setPower(0.79f * leftStickY);

        }

        if(gamepad2.b){

            robot.centerBallKnocker();
            robot.bringUpBallKnocker();

        }

        if(gamepad2.y && !yButton2PressedLastTime){

            relicMode = !relicMode;
            yButton2PressedLastTime = true;

        }

        if(gamepad2.y){

            yButton2PressedLastTime = true;

        }

        else{

            yButton2PressedLastTime = false;

        }

        if(gamepad1.y && !yButton1PressedLastTime){

            superSlowMode = !superSlowMode;
            yButton1PressedLastTime = true;

        }

        if(gamepad1.y){

            yButton1PressedLastTime = true;

        }

        else{

            yButton1PressedLastTime = false;

        }

        if(gamepad1.right_bumper && !rightBumperPressedLastTime){

            braking = !braking;

            if(braking){

                robot.setBrakes(true);

            }

            else{

                robot.setBrakes(false);

            }

        }

        if(gamepad1.right_bumper){

            rightBumperPressedLastTime = true;

        }

        else{

            rightBumperPressedLastTime = false;

        }

        float frInputs = 0;
        float flInputs = 0;
        float brInputs = 0;
        float blInputs = 0;

        float creep = gamepad1.left_trigger;
        float speedMultiplier = 0.7f;
        float multiplier = 1f - (creep * 0.55f * speedMultiplier);
        multiplier *= (gamepad1.left_bumper ? 1f : speedMultiplier); // Skert mode

        if(creep > 0.94f && gamepad1.left_bumper){

            multiplier = 0.17f;

        }

        if(superSlowMode){

            multiplier = 0.17f * (gamepad1.left_bumper ? 2.45f : 1f);

        }

        if(!controlsReversed){

            // Forward and backwards
            frInputs += gamepad1.right_stick_y;
            flInputs += gamepad1.right_stick_y;
            brInputs += gamepad1.right_stick_y;
            blInputs += gamepad1.right_stick_y;

            // Strafing
            frInputs += gamepad1.right_stick_x;
            flInputs -= gamepad1.right_stick_x;
            brInputs -= gamepad1.right_stick_x;
            blInputs += gamepad1.right_stick_x;

        }

        else{

            // Forward and backwards
            frInputs -= gamepad1.right_stick_y;
            flInputs -= gamepad1.right_stick_y;
            brInputs -= gamepad1.right_stick_y;
            blInputs -= gamepad1.right_stick_y;

            // Strafing
            frInputs -= gamepad1.right_stick_x;
            flInputs += gamepad1.right_stick_x;
            brInputs += gamepad1.right_stick_x;
            blInputs -= gamepad1.right_stick_x;

        }

        // Skid steering
        frInputs -= gamepad1.left_stick_x;
        brInputs -= gamepad1.left_stick_x;
        flInputs += gamepad1.left_stick_x;
        blInputs += gamepad1.left_stick_x;


        frInputs *= multiplier;
        flInputs *= multiplier;
        brInputs *= multiplier;
        blInputs *= multiplier;

        // Reversing the controls
        if(gamepad1.b && !bButtonPressedLastTime){

            controlsReversed = !controlsReversed;
            bButtonPressedLastTime = true;

        }

        if(!gamepad1.b){

            bButtonPressedLastTime = false;

        }

        robot.frontRight.setPower(frInputs);
        robot.frontLeft.setPower(flInputs);
        robot.backRight.setPower(brInputs);
        robot.backLeft.setPower(blInputs);

        if(autoFlipperUp && autoFlipperUpTimer.milliseconds() <= 700){

            robot.flipperMotor.setPower(-0.7f);

        }

        else if(autoFlipperUpTimer.milliseconds() > 700){

            autoFlipperUp = false;

        }

        robot.centerBallKnocker();
        robot.bringUpBallKnocker();

        robot.relicFlipper.setPosition(relicFlipperPosition);

        robot.relicGripper.setPosition(relicGrabberPosition);

        float right_trigger = gamepad1.right_trigger;
        float defaultPos = 0.6f - (braking ? 0.03f : 0f);
        float range = defaultPos - 0.28f;
        float finalPos = defaultPos - range * right_trigger;

        robot.ballKnocker.setPosition(finalPos);

        telemetry.update();

    }

    @Override
    public void stop(){

        robot.initBallKnocker();

    }

}
