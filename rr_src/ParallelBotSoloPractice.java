package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
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

@TeleOp(name="ParallelBot TeleOp SOLO PRACTICE", group="ParallelBot")
public class ParallelBotSoloPractice extends OpMode {

    private ParallelBotHardware robot = new ParallelBotHardware();

    private boolean controlsReversed = false; // Reverses controls for far box
    private boolean relicMode = false; // Relic mode for gp2
    private boolean superSlowMode = false; // Relic slow mode for driving (higher expo)

    private final float DEFAULT_EXPO = 1.9f;
    private final float SUPER_SLOW_EXPO = 3f;

    private final float SLOW_STRAFE_MULTIPLIER = 0.5f;
    private final float TURNING_SPEED = 0.8f;

    private float relicFlipperPosition; // Keeps the position of the relic flipper
    private float relicGrabberPosition; // Keeps the position of the relic grabber

    private Gamepad previousGamepad1; // Previous gamepads
    private Gamepad previousGamepad2;

    private boolean resettingFlipper = false; // Auto reset stuff for the flipper
    private ElapsedTime resetTimer = new ElapsedTime();

    private boolean autoFlipperUp = false;
    private ElapsedTime autoFlipperUpCooldown = new ElapsedTime();

    private boolean keepingUpLift = false;

    private boolean jewelKnockerDown = false;

    private boolean flipperGripperOpen = true;
    private boolean bothStickButtonsLastTime = false;

    @Override
    public void init () {

        robot.init(hardwareMap, false);
        robot.bringUpBallKnocker();

        robot.rightHarvesterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftHarvesterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Inited");
        telemetry.update();

    }

    @Override
    public void start(){

        robot.initMovements();

        robot.parkingBrake.setPosition(0.189f);

        previousGamepad1 = gamepad1;
        previousGamepad2 = gamepad2;

        relicFlipperPosition = robot.partiallyRetractRelicFlipper(relicMode);
        relicGrabberPosition = robot.openRelicGripper();

        resettingFlipper = true;
        resetTimer.reset();

    }

    @Override
    public void loop(){

        // Parking brake
        checkParkingBrake();

        // Bring the harvester up or down for parking
        checkHarvesterUpDown();

        if(resetTimer.milliseconds() > 900){

            resettingFlipper = false;

        }

        float rightStickY = gamepad2.right_stick_y;
        float leftStickY = gamepad2.left_stick_y;

        double liftPower = leftStickY * 0.5f;

        liftPower = clamp(liftPower, -0.5f, 0.5f);
        liftPower += (keepingUpLift ? -0.5f : 0.15f);

        if(gamepad1.right_stick_button && autoFlipperUpCooldown.milliseconds() > 400){

            autoFlipperUp = true;

        }

        if(gamepad1.left_stick_button){

            autoFlipperUp = false;

        }

        if(gamepad1.dpad_up){

            keepingUpLift = true;

        }

        if(gamepad1.dpad_down){

            keepingUpLift = false;

        }

        telemetry.addData("afu", autoFlipperUp);

        double flipperPower = rightStickY * (gamepad2.right_stick_button ? 1f : 0.75f);
        flipperPower -= (autoFlipperUp ? 0.95f : 0f);
        flipperPower += (gamepad1.left_stick_button ? 0.6f : 0f);
        flipperPower = clamp(flipperPower, -1f, 1f);

        telemetry.addData("pow", flipperPower);

        // Let the driver also enter park mode
        if(gamepad1.x){

            robot.setBrakes(true);

        }

        // Harvester controls
        double rightHarvesterPower = gamepad2.right_trigger * (gamepad2.right_bumper ? -1f : 1f) * 0.8f;
        double leftHarvesterPower = gamepad2.left_trigger * (gamepad2.left_bumper ? -1f : 1f) * 0.8f;

        rightHarvesterPower += (gamepad1.right_bumper ? -1f : gamepad1.right_trigger) * 0.8f;
        leftHarvesterPower += (gamepad1.left_bumper ? -1f : gamepad1.left_trigger) * 0.8f;

        rightHarvesterPower = clamp(rightHarvesterPower, -0.8f, 0.8f);
        leftHarvesterPower = clamp(leftHarvesterPower, -0.8f, 0.8f);

        // Harvester controls
        robot.rightHarvesterMotor.setPower(rightHarvesterPower);
        robot.leftHarvesterMotor.setPower(leftHarvesterPower);

        // Toggle the relic mode
        if(gamepad2.y && !previousGamepad2.y){

            relicMode = !relicMode;

        }

        if(gamepad2.x && !previousGamepad2.x){

            robot.setBrakes(!(robot.frontRight.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE));

        }

        // Toggle super slow mode (higher expo)
        if(gamepad1.y && !previousGamepad1.y){

            superSlowMode = !superSlowMode;

        }

        // Calculate the expo and clamped inputs
        double rsy = gamepad1.right_stick_y;
        boolean rsyPos = rsy > 0;

        double rsx = gamepad1.right_stick_x;
        boolean rsxPos = rsx > 0;

        double lsx = gamepad1.left_stick_x;
        boolean lsxPos = lsx > 0;

        // Apply the expo with everything positive
        rsy = Math.abs(Math.pow(Math.abs(rsy), superSlowMode ? SUPER_SLOW_EXPO : DEFAULT_EXPO));
        rsx = Math.abs(Math.pow(Math.abs(rsx), superSlowMode ? SUPER_SLOW_EXPO : DEFAULT_EXPO));
        lsx = Math.abs(Math.pow(Math.abs(lsx), superSlowMode ? SUPER_SLOW_EXPO : DEFAULT_EXPO));

        // Make sure everything is the right sign
        rsy = (rsyPos ? 1f : -1f) * rsy;
        rsx = (rsxPos ? 1f : -1f) * rsx;
        lsx = (lsxPos ? 1f : -1f) * lsx;

        // Clamp the values
        float min = superSlowMode ? -0.6f : -1f;
        float max = superSlowMode ? 0.6f : 1f;

        rsy = clamp(rsy, min, max);
        rsx = clamp(rsx, min, max);
        lsx = clamp(lsx, min, max);

        float frInputs = 0;
        float flInputs = 0;
        float brInputs = 0;
        float blInputs = 0;

        if(!controlsReversed){

            // Forward and backwards
            frInputs += rsy;
            flInputs += rsy;
            brInputs += rsy;
            blInputs += rsy;

            // Strafing
            frInputs += rsx * (gamepad1.right_stick_button ? 1f : SLOW_STRAFE_MULTIPLIER);
            flInputs -= rsx * (gamepad1.right_stick_button ? 1f : SLOW_STRAFE_MULTIPLIER);
            brInputs -= rsx * (gamepad1.right_stick_button ? 1f : SLOW_STRAFE_MULTIPLIER);
            blInputs += rsx * (gamepad1.right_stick_button ? 1f : SLOW_STRAFE_MULTIPLIER);

        }

        else{

            // Forward and backwards
            frInputs -= rsy;
            flInputs -= rsy;
            brInputs -= rsy;
            blInputs -= rsy;

            // Strafing
            frInputs -= rsx * (gamepad1.right_stick_button ? 1f : SLOW_STRAFE_MULTIPLIER);
            flInputs += rsx * (gamepad1.right_stick_button ? 1f : SLOW_STRAFE_MULTIPLIER);
            brInputs += rsx * (gamepad1.right_stick_button ? 1f : SLOW_STRAFE_MULTIPLIER);
            blInputs -= rsx * (gamepad1.right_stick_button ? 1f : SLOW_STRAFE_MULTIPLIER);

        }

        // Skid steering
        frInputs -= lsx * TURNING_SPEED;
        brInputs -= lsx * TURNING_SPEED;
        flInputs += lsx * TURNING_SPEED;
        blInputs += lsx * TURNING_SPEED;

        robot.frontRight.setPower(frInputs);
        robot.frontLeft.setPower(flInputs);
        robot.backRight.setPower(brInputs);
        robot.backLeft.setPower(blInputs);

        if((gamepad1.right_stick_button && gamepad1.left_stick_button) && !bothStickButtonsLastTime){

            flipperGripperOpen = !flipperGripperOpen;
            autoFlipperUpCooldown.reset();

        }

        if(gamepad1.right_stick_button && gamepad1.left_stick_button){

            bothStickButtonsLastTime = true;

        }

        else{

            bothStickButtonsLastTime = false;

        }

        // If not in relic mode, move the lift
        if(!relicMode){

            if((!robot.limitSwitch.getState() || !robot.limitSwitch2.getState()) && liftPower > 0f){

                liftPower = 0f;

            }

            robot.liftMotor.setPower(liftPower);

            // If not in relic mode and not doing any auto moving, move flipper
            if(!resettingFlipper){

                robot.flipperMotor.setPower(flipperPower);

            }

        }

        // Do auto flipper up or stop it


        // Relic mode controls
        if(relicMode){

            // Open and close the gripper
            if(gamepad2.x){

                relicGrabberPosition = robot.openRelicGripper();

            }

            if(gamepad2.a){

                relicGrabberPosition = robot.closeRelicGripper();

            }

            // Manual control of the gripper
            if(gamepad2.dpad_right){

                double newPos = robot.relicGripper.getPosition() + 0.01f;
                robot.relicGripper.setPosition(newPos);
                relicGrabberPosition = (float) (newPos);

            }

            if(gamepad2.dpad_left){

                double newPos = robot.relicGripper.getPosition() - 0.01f;
                robot.relicGripper.setPosition(newPos);
                relicGrabberPosition = (float) (newPos);

            }

            // Retract and extend relic flipper
            if(gamepad2.left_bumper){

                relicFlipperPosition = robot.partiallyRetractRelicFlipper(relicMode);

            }

            if(gamepad2.right_bumper){

                relicFlipperPosition = robot.extendRelicFlipper();

            }

            // Manually control the relic flipper
            robot.relicFlipper.setPosition(robot.relicFlipper.getPosition() + 0.0015f * rightStickY);
            relicFlipperPosition = (float) (robot.relicFlipper.getPosition() + 0.0015f * rightStickY);

            // Control the extension
            robot.extender.setPower(0.79f * leftStickY);

        }

        // Reversing the controls
        if(gamepad1.b && !previousGamepad1.b){

            controlsReversed = !controlsReversed;

        }

        // Make sure to update servo pos every loop
        robot.centerBallKnocker();

        robot.relicFlipper.setPosition(relicFlipperPosition);
        robot.relicGripper.setPosition(relicGrabberPosition);

        // Set the ball knocker position
        if(gamepad1.a && !previousGamepad1.a){

            jewelKnockerDown = !jewelKnockerDown;

        }

        if(jewelKnockerDown){

            robot.bringDownBallKnocker();

        }

        else{

            robot.bringUpBallKnocker();

        }

        if(flipperGripperOpen){

            robot.openFlipperGripper();

        }

        else{

            robot.closeFlipperGripper();

        }

        previousGamepad1 = new Gamepad();
        previousGamepad1.b = gamepad1.b;
        previousGamepad1.a = gamepad1.a;
        previousGamepad1.y = gamepad1.y;
        previousGamepad1.left_stick_button = gamepad1.left_stick_button;
        previousGamepad1.right_stick_button = gamepad1.right_stick_button;

        previousGamepad2 = new Gamepad();
        previousGamepad2.y = gamepad2.y;
        previousGamepad2.x = gamepad2.x;

        telemetry.update();

    }

    @Override
    public void stop(){

        robot.initBallKnocker();

    }

    public double clamp(double val, double min, double max){

        return Math.max(min, Math.min(max, val));

    }

    public void checkParkingBrake(){

        if(gamepad1.dpad_right){

            robot.parkingBrake.setPosition(0.189f);

        }

        if(gamepad1.dpad_left){

            robot.parkingBrake.setPosition(0.628f);

        }

    }

    public void checkHarvesterUpDown(){

        if(gamepad1.dpad_up){

            // robot.harvesterUp();

        }

        if(gamepad1.dpad_down){

            // robot.harvesterDown();

        }

    }

}
