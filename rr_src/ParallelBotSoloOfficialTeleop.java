package org.firstinspires.ftc.teamcode.kssstuff;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ParallelBotHardware;

@TeleOp(name="ParallelBot TeleOp OFFICIAL SOLO TELEOP", group="ParallelBot")
public class ParallelBotSoloOfficialTeleop extends OpMode {

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

    private boolean autoFlipperUp = false;
    private boolean keepingUpLift = false;
    private boolean flipperGripperOpen = true;

    private boolean jewelKnockerDown = false;

    private ToggleGamepad toggleGamepad1;

    private int flipperState = 0; // 0 = down, 1 = up and gripping, 2 = up and not gripping
    // if in state 0 and holding down, it will bring the lift down faster

    @Override
    public void init () {

        robot.init(hardwareMap, false);
        robot.bringUpBallKnocker();

        toggleGamepad1 = new ToggleGamepad();

        robot.rightHarvesterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftHarvesterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Inited");
        telemetry.update();

    }

    @Override
    public void start(){

        robot.initMovements();

        robot.parkingBrake.setPosition(0.189f);

        relicFlipperPosition = robot.partiallyRetractRelicFlipper(relicMode);
        relicGrabberPosition = robot.openRelicGripper();

    }

    @Override
    public void loop(){

        toggleGamepad1.update(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y, gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.left_stick_button, gamepad1.right_stick_button, gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right);

        // Parking brake
        checkParkingBrake();

        // Bring the harvester up or down for parking
        checkHarvesterUpDown();

        // Reversing the controls
        if(toggleGamepad1.debouncedGamepadb){

            controlsReversed = !controlsReversed;

        }

        // Check all the drivetrain inputs
        checkDrivetrain();

        // Set the lift power (this will heat the motor up a bit but who cares)
        double liftPower = (keepingUpLift ? -0.4f : 0.15f);
        liftPower = relicMode ? 0f : liftPower; // Disable in relic mode

        // Guard against stripping stuff too bad
        if((!robot.limitSwitch.getState() || !robot.limitSwitch2.getState()) && liftPower > 0f){

            liftPower = 0f;

        }

        robot.liftMotor.setPower(liftPower);

        // Toggle through the flipper states if not in relic mode
        if(toggleGamepad1.debouncedGamepadright_stick_button && !relicMode){

            flipperState = (flipperState + 1) % 3;

        }

        switch (flipperState) {
            case 0:
                autoFlipperUp = false;
                flipperGripperOpen = false;
                break;
            case 1:
                autoFlipperUp = true;
                flipperGripperOpen = false;
                break;
            case 2:
                autoFlipperUp = true;
                flipperGripperOpen = true;
                break;
            default:
                telemetry.addLine("Code broke!");
                 break;
        }

        if(toggleGamepad1.debouncedGamepadleft_stick_button){

            keepingUpLift = !keepingUpLift;
            flipperGripperOpen = !keepingUpLift;

        }

        telemetry.addData("gp1 right stick button", gamepad1.right_stick_button);
        telemetry.addData("toggle gp1 right stick button", toggleGamepad1.debouncedGamepadright_stick_button);

        // Set flipper power
        double flipperPower = -(autoFlipperUp ? 0.95f : 0f); // If auto flipping up, get it up
        flipperPower += (gamepad1.right_stick_button && flipperState == 0 ? 0.6f : 0f); // If bringing it down, give it some speed but not too much
        flipperPower = clamp(flipperPower, -1f, 1f); // Clamp
        flipperPower = relicMode ? 0f : flipperPower; // Only when not in relic mode

        robot.flipperMotor.setPower(flipperPower);

        telemetry.addData("Flipper Power", flipperPower);
        telemetry.addData("Auto Flipper Up", autoFlipperUp);
        telemetry.addData("Keeping Lift Up", keepingUpLift);
        telemetry.addData("Lift Power", liftPower);
        telemetry.addData("Flipper State", flipperState);

        // Toggle brakes
        if(toggleGamepad1.debouncedGamepadx && !relicMode){

            robot.setBrakes(!(robot.frontRight.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE));

        }

        // Toggle the relic mode and superSlowMode along with it
        if(toggleGamepad1.debouncedGamepady){

            relicMode = !relicMode;
            superSlowMode = relicMode;

            if(relicMode){

                flipperState = 0;

            }

        }

        // Set the ball knocker position
        if(toggleGamepad1.debouncedGamepada && !relicMode){

            jewelKnockerDown = !jewelKnockerDown;

        }

        // Harvester controls
        double rightHarvesterPower = (gamepad1.right_bumper ? -1f : gamepad1.right_trigger) * 0.8f;
        double leftHarvesterPower = (gamepad1.left_bumper ? -1f : gamepad1.left_trigger) * 0.8f;

        if(relicMode){

            rightHarvesterPower = gamepad1.dpad_up ? 0.8f : 0.0f;
            leftHarvesterPower = gamepad1.dpad_up ? 0.8f : 0.0f;

            rightHarvesterPower = gamepad1.dpad_down ? -0.8f : 0.0f;
            leftHarvesterPower = gamepad1.dpad_down ? -0.8f : 0.0f;

        }

        rightHarvesterPower = clamp(rightHarvesterPower, -0.8f, 0.8f);
        leftHarvesterPower = clamp(leftHarvesterPower, -0.8f, 0.8f);

        robot.rightHarvesterMotor.setPower(rightHarvesterPower);
        robot.leftHarvesterMotor.setPower(leftHarvesterPower);

        // Relic mode controls
        if(relicMode){

            // Open and close the gripper
            if(gamepad1.x){

                relicGrabberPosition = robot.openRelicGripper();

            }

            if(gamepad1.a){

                relicGrabberPosition = robot.closeRelicGripper();

            }

            // Manual control of the gripper
            if(gamepad1.dpad_right){

                double newPos = robot.relicGripper.getPosition() + 0.01f;
                robot.relicGripper.setPosition(newPos);
                relicGrabberPosition = (float) (newPos);

            }

            if(gamepad1.dpad_left){

                double newPos = robot.relicGripper.getPosition() - 0.01f;
                robot.relicGripper.setPosition(newPos);
                relicGrabberPosition = (float) (newPos);

            }

            // Retract and extend relic flipper
            if(gamepad1.left_bumper){

                relicFlipperPosition = robot.partiallyRetractRelicFlipper(relicMode);

            }

            if(gamepad1.right_bumper){

                relicFlipperPosition = robot.extendRelicFlipper();

            }

            // Manually control the relic flipper with the bumpers
            robot.relicFlipper.setPosition(robot.relicFlipper.getPosition() + 0.0015f * (gamepad1.right_trigger - gamepad1.left_trigger));
            relicFlipperPosition = (float) (robot.relicFlipper.getPosition() + 0.0015f * (gamepad1.right_trigger - gamepad1.left_trigger));

            // Control the extension
            robot.extender.setPower(0.79f * ((gamepad1.right_stick_button ? 1 : 0) - (gamepad1.left_stick_button ? 1 : 0)));

        }

        // Make sure to update servo pos every loop
        robot.centerBallKnocker();

        robot.relicFlipper.setPosition(relicFlipperPosition);
        robot.relicGripper.setPosition(relicGrabberPosition);

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

        telemetry.update();

    }

    @Override
    public void stop(){

        robot.initBallKnocker();

    }

    public double clamp(double val, double min, double max){

        return Math.max(min, Math.min(max, val));

    }

    public void checkDrivetrain(){

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

    }

    public void checkParkingBrake(){

        if(gamepad1.dpad_right && !relicMode){

            robot.parkingBrake.setPosition(0.189f);

        }

        if(gamepad1.dpad_left && !relicMode){

            robot.parkingBrake.setPosition(0.628f);

        }

    }

    public void checkHarvesterUpDown(){

        if(gamepad1.dpad_up && !relicMode){

            robot.harvesterUp();

        }

        if(gamepad1.dpad_down && !relicMode){

            robot.harvesterDown();

        }

    }

}
