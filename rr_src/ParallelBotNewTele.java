package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

@TeleOp(name="ParallelBot TeleOp NEW", group="ParallelBot")
public class ParallelBotNewTele extends OpMode {

    private ParallelBotHardware robot = new ParallelBotHardware();

    private boolean controlsReversed = false; // Reverses controls for far box
    private boolean relicMode = false; // Relic mode for gp2
    private boolean superSlowMode = false; // Relic slow mode for driving (higher expo)

    private boolean gp2yp = false;
    private boolean gp1xp = false;

    private final float DEFAULT_EXPO = 1.9f;
    private final float SUPER_SLOW_EXPO = 3f;

    private final float SLOW_STRAFE_MULTIPLIER = 0.5f;
    private final float TURNING_SPEED = 0.45f;

    private float relicFlipperPosition; // Keeps the position of the relic flipper
    private float relicGrabberPosition; // Keeps the position of the relic grabber

    private Gamepad previousGamepad1; // Previous gamepads
    private Gamepad previousGamepad2;

    private BNO055IMU imu;

    private ElapsedTime autoFlipperUpTimer = new ElapsedTime(); // Auto up for the flipper
    private boolean autoFlipperUp = false;
    private boolean keepingFlipperUp = false;

    private boolean jewelKnockerDown = false;

    private boolean flipperGripperOpen = true;
    private boolean a2p = false;
    private boolean gp2bp = false;

    private DcMotorEx flipperMotorEx;

    private DcMotorEx rightHarvesterMotorEx;
    private DcMotorEx leftHarvesterMotorEx;

    private BNO055IMU.Parameters parameters;

    private ServoImplEx relicFlipperEx;

    private int consecutiveTimeFlipperNotMoving = 0;

    @Override
    public void init () {

        robot.init(hardwareMap, false);
        robot.bringUpBallKnocker();

        robot.rightHarvesterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftHarvesterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightHarvesterMotorEx = (DcMotorEx) robot.rightHarvesterMotor;
        leftHarvesterMotorEx = (DcMotorEx) robot.leftHarvesterMotor;

        robot.flipperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flipperMotorEx = (DcMotorEx) robot.flipperMotor;

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        relicFlipperEx = (ServoImplEx) robot.relicFlipper;
        relicFlipperEx.setPwmEnable();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

    }

    @Override
    public void loop(){

        if(gamepad2.a && !a2p && !relicMode){

            flipperGripperOpen = !flipperGripperOpen;

        }

        a2p = gamepad2.a;

        // Parking brake
        checkParkingBrake();

        // Bring the harvester up or down for parking
        checkHarvesterUpDown();

        float rightStickY = gamepad2.right_stick_y;
        float leftStickY = gamepad2.left_stick_y;

        if(Math.abs(flipperMotorEx.getVelocity(AngleUnit.DEGREES)) < 5 && rightStickY > 0){

            consecutiveTimeFlipperNotMoving++;

        }

        else{

            consecutiveTimeFlipperNotMoving = 0;

        }

        if(consecutiveTimeFlipperNotMoving > 5 || gamepad2.left_stick_button){

            // robot.flipperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // robot.flipperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        telemetry.addData("consec", consecutiveTimeFlipperNotMoving);

        double liftPower = leftStickY * 0.5f;

        liftPower = clamp(liftPower, -0.5f, 0.5f);

        boolean applySlowDown = !gamepad2.right_stick_button && Math.abs(robot.flipperMotor.getCurrentPosition()) > 60;
        applySlowDown = false;

        double flipperPower = rightStickY * (rightStickY > 0 ? 0.6f : 1f) * (applySlowDown ? 0.6f : 1f);

        telemetry.addData("fp", flipperPower);

        if(Math.abs(rightStickY) > 0.18 && !relicMode){

            keepingFlipperUp = false;

        }

        // Let the driver also enter park mode
        if(gamepad1.x && !gp1xp){

            startAutoFlipperUp();
            robot.setBrakes(true);

        }

        gp1xp = gamepad1.x;

        if(gamepad2.b && !gp2bp && !relicMode){

            // startAutoFlipperUp();

        }

        gp2bp = gamepad2.b;

        // Harvester controls
        double rightHarvesterPower = gamepad2.right_trigger * (gamepad2.right_bumper ? -1f : 1f) * 0.9f;
        double leftHarvesterPower = gamepad2.left_trigger * (gamepad2.left_bumper ? -1f : 1f) * 0.9f;

        /*if(Math.abs(robot.liftMotor.getCurrentPosition()) < 25 && !relicMode){

            rightHarvesterPower += -liftPower * 0.8f;
            leftHarvesterPower += -liftPower * 0.8f;

        }

        if(Math.abs(robot.flipperMotor.getCurrentPosition()) < 30 && !relicMode){

            rightHarvesterPower += -flipperPower * 0.6f;
            leftHarvesterPower += -flipperPower * 0.6f;

        }*/

        telemetry.addData("memes", robot.flipperMotor.getCurrentPosition());

        rightHarvesterPower = clamp(rightHarvesterPower, -0.9f, 0.9f);
        leftHarvesterPower = clamp(leftHarvesterPower, -0.9f, 0.9f);

        // Harvester controls
        robot.rightHarvesterMotor.setPower(rightHarvesterPower);
        robot.leftHarvesterMotor.setPower(leftHarvesterPower);
        // rightHarvesterMotorEx.setVelocity(rightHarvesterPower * 1000f, AngleUnit.DEGREES);
        // leftHarvesterMotorEx.setVelocity(leftHarvesterPower * 1000f, AngleUnit.DEGREES);

        telemetry.addData("harv", rightHarvesterMotorEx.getVelocity(AngleUnit.DEGREES));

        // Toggle the relic mode
        if(gamepad2.y && !gp2yp){

            relicMode = !relicMode;

        }

        // Toggle super slow mode (higher expo)
        if(gamepad1.y && !previousGamepad1.y){

            superSlowMode = !superSlowMode;

        }

        gp2yp = gamepad2.y;

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
        float min = superSlowMode ? -0.4f : -1f;
        float max = superSlowMode ? 0.4f : 1f;

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
        frInputs -= lsx * TURNING_SPEED * (superSlowMode ? 1.4f : 1f);
        brInputs -= lsx * TURNING_SPEED * (superSlowMode ? 1.4f : 1f);
        flInputs += lsx * TURNING_SPEED * (superSlowMode ? 1.4f : 1f);
        blInputs += lsx * TURNING_SPEED * (superSlowMode ? 1.4f : 1f);

        if(gamepad1.left_bumper){

            imu.initialize(parameters);

        }

        if(gamepad1.left_stick_button){

            // Do the auto gyro line up
            float minSpeed = 0.4f;
            float maxSpeed = 1f;
            float range = maxSpeed - minSpeed;
            float angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            float percentDone = 1 - (Math.abs(angle) / 180f);
            double turnPower = (angle > 0 ? 1f : -1f) * (0.8f - (percentDone * range));

            telemetry.addData("percent done", percentDone);
            telemetry.addData("turn power", turnPower);

            if(percentDone < 0.98f){

                frInputs -= turnPower;
                brInputs -= turnPower;
                flInputs += turnPower;
                blInputs += turnPower;

            }

        }

        robot.frontRight.setPower(frInputs);
        robot.frontLeft.setPower(flInputs);
        robot.backRight.setPower(brInputs);
        robot.backLeft.setPower(blInputs);

        // If not in relic mode, move the lift
        if(!relicMode){

            if((!robot.limitSwitch.getState() || !robot.limitSwitch2.getState()) && liftPower > 0f){

                liftPower = 0f;

                if(Math.abs(robot.liftMotor.getCurrentPosition()) > 1){

                    // robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    // robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                }

            }

            telemetry.addData("lm", robot.liftMotor.getCurrentPosition());

            robot.liftMotor.setPower(liftPower);

            // If not in relic mode and not doing any auto moving, move flipper
            if(!autoFlipperUp && !keepingFlipperUp){

                robot.flipperMotor.setPower(flipperPower);

            }

            else if(keepingFlipperUp){

                robot.flipperMotor.setPower(-0.4f);

            }

            telemetry.addData("flipper speed", flipperMotorEx.getVelocity(AngleUnit.DEGREES));

        }

        // Do auto flipper up or stop it
        if(autoFlipperUp && autoFlipperUpTimer.milliseconds() <= 1400){



        }

        else if((autoFlipperUpTimer.milliseconds() > 2000 || !robot.flipperMotor.isBusy()) && autoFlipperUp){

            autoFlipperUp = false;
            keepingFlipperUp = true;
            robot.flipperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

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

            telemetry.addData("FLIPPY MEMES", robot.relicFlipper.getPosition());

            // Manually control the relic flipper
            robot.relicFlipper.setPosition(robot.relicFlipper.getPosition() + 0.003f * rightStickY);
            relicFlipperPosition = (float) (robot.relicFlipper.getPosition() + 0.003f * rightStickY);

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

        if(superSlowMode){

            robot.bringUpBallKnockerLots();

        }

        else if(jewelKnockerDown){

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

        telemetry.addData("k", keepingFlipperUp);
        telemetry.addData("adfjkhsd", flipperGripperOpen);

        previousGamepad1 = new Gamepad();
        previousGamepad1.b = gamepad1.b;
        previousGamepad1.a = gamepad1.a;
        previousGamepad1.y = gamepad1.y;
        previousGamepad1.left_stick_button = gamepad1.left_stick_button;
        previousGamepad1.right_stick_button = gamepad1.right_stick_button;

        previousGamepad2 = new Gamepad();
        previousGamepad2.y = gamepad2.y;
        previousGamepad2.a = gamepad2.a;

        telemetry.update();

    }

    @Override
    public void stop(){

        relicFlipperEx.setPwmDisable();
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

    public void startAutoFlipperUp(){

        autoFlipperUp = true;
        autoFlipperUpTimer.reset();
        robot.flipperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.flipperMotor.setTargetPosition(-80);
        robot.flipperMotor.setPower(-1f);

    }

    public void checkHarvesterUpDown(){

        if(gamepad1.dpad_up){

            robot.harvesterUp();

        }

        if(gamepad1.dpad_down){

            robot.harvesterDown();

        }

    }

}
