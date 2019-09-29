package org.firstinspires.ftc.teamcode;

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

public abstract class ParallelBotAutoWithFlexCorner extends BaseAutonomous {

    // Left, right, center

    public ParallelBotAutoWithFlexCorner(boolean color){

        super(color);

    }

    @Override
    public void runOpMode(){

        try{

            super.runOpMode();

            // Engage the brakes
            robot.setBrakes(true);

            // Bring up the relic flipper
            robot.partiallyRetractRelicFlipper();

            robot.flipperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Activate Vuforia
            relicTrackables.activate();

            // Take a reading of the pictograph
            countVisibleImages();

            knockJewel();

            float motorPower = 0.45f;

            // Init the gyro
            imu.initialize(gyroParameters);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // Move forward and read the pictograph two more times
            moveForwardUsingEncoder((isBlue ? -1 : 1) * 150, 500, (isBlue ? -1f : 1f) * motorPower);
            countVisibleImages();
            moveForwardUsingEncoder((isBlue ? -1 : 1) * 200, 500, (isBlue ? -1f : 1f) * motorPower);
            countVisibleImages();
            moveForwardUsingEncoder((isBlue ? -1 : 1) * 500, 1000, (isBlue ? -1f : 1f) * motorPower);

            // Calculate the visible image
            int visibleImage = calculateVisibleImage();

            telemetry.addData("Visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
            telemetry.update();

            robot.setPower(0f);

            // Get the starting angle
            float turnPower = 0.35f;
            float startingAngle = angles.firstAngle;

            // If its blue, do a 180 turn
            if(isBlue){

                // Turn
                gyroTurn(turnPower, startingAngle, 176, 6500);

            }

            else{

                gyroSquareUp(turnPower, startingAngle, 3000);

            }

            // In case limp
            robot.bringUpBallKnocker();

            // Set the encoder ticks and power
            float strafePower = -0.45f;
            int encoderTicks = 0;

            if(visibleImage == 1){

                encoderTicks = (isBlue ? 840 : 345);

            }

            if(visibleImage == 0){

                encoderTicks = (isBlue ? 260 : 990);

            }

            if(visibleImage == 2){

                encoderTicks = (isBlue ? 505 : 630);

            }

            strafePower *= (isBlue ? -1 : 1);

            // Stafe to the column
            moveUsingEncoder((isBlue ? -1 : 1) * encoderTicks, 2000, -strafePower, strafePower, strafePower, -strafePower);

            // Move forward then back a little
            moveForwardUsingEncoder(700, 1000, 0.3f);
            float squaredAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            moveForwardUsingEncoder(-120, 1000, -0.1f);

            // In case limp
            robot.bringUpBallKnocker();
            robot.partiallyRetractRelicFlipper();

            // Flip the glyph in
            robot.flipperMotor.setPower(-1f);
            sleep(1350);

            // Back up a little
            moveForwardUsingEncoder(-180, 1000, -0.5f);

            // In case limp
            robot.partiallyRetractRelicFlipper();

            // bring flipper down
            robot.flipperMotor.setPower(0.4f);

            // Push the glyph in
            moveForwardUsingEncoder(300, 2000, 0.3f);
            moveForwardUsingEncoder(-200, 2000, -0.3f);
            robot.flipperMotor.setPower(0f);

            // Turn towards the glyph pit
            turnPower = (isBlue ? -1f : 1f) * -0.35f;
            encoderTicks = 55;
            encoderTicks = (!isBlue && visibleImage == 0 ? 5 : encoderTicks);
            encoderTicks = (isBlue && visibleImage == 1 ? 5 : encoderTicks);

            moveUsingEncoder((isBlue ? -1 : 1) * encoderTicks, 1000, turnPower, -turnPower, turnPower, -turnPower);

            // In case limp
            robot.bringUpBallKnocker();

            // Move to the glyph pit
            moveForwardUsingEncoder(-1000, 3000, -1f);

            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Get ready to harvest
            float driveSpeed = -0.13f;
            robot.setPower(driveSpeed);

            rightHarvesterMotorEx.setVelocity(800, AngleUnit.DEGREES);
            leftHarvesterMotorEx.setVelocity(800, AngleUnit.DEGREES);
            ElapsedTime time = new ElapsedTime();
            time.reset();

            ElapsedTime collectingTimer = new ElapsedTime();
            int state = 0; // 0 = Regular, 3 = Jammed
            int consecutiveTimesSeenGlyph = 0;
            int unjammingMode = 0; // 0 = Left goes unpowered, 1 = Right goes unpowered, 2 = spit out

            // While not timed out and we don't have 3 glyphs
            while(opModeIsActive() && time.milliseconds() < 4000 && consecutiveTimesSeenGlyph < 2){

                telemetry.addData("pos", robot.frontRight.getCurrentPosition());
                telemetry.update();

                if(robot.glyphSensor.getLightDetected() > 0.55f){

                    consecutiveTimesSeenGlyph++;

                }

                else{

                    consecutiveTimesSeenGlyph = 0;

                }

                // Figure out if jammed by checking the encoders
                boolean jammed = rightHarvesterMotorEx.getVelocity(AngleUnit.DEGREES) < 200 && leftHarvesterMotorEx.getVelocity(AngleUnit.DEGREES) < 200;
                // Also include a number of cooldown conditions to account for slow acceleration
                boolean cooldownConditionsMet = time.milliseconds() > 200 && state != 3 && collectingTimer.milliseconds() > 1000;

                // If jammed, rotate through the unjamming modes
                if(jammed && cooldownConditionsMet){

                    state = 3;
                    collectingTimer.reset();
                    robot.setPower(0.1f);
                    unjammingMode = (unjammingMode + 1) % 3;

                }

                // If jammed, do the unjamming modes
                if(state == 3){

                    if(unjammingMode == 3){

                        rightHarvesterMotorEx.setVelocity(800, AngleUnit.DEGREES);
                        leftHarvesterMotorEx.setVelocity(0, AngleUnit.DEGREES);

                        if(collectingTimer.milliseconds() > 400){

                            collectingTimer.reset();
                            state = 0;
                            robot.setPower(driveSpeed);

                        }

                    }

                    else if(unjammingMode == 2){

                        rightHarvesterMotorEx.setVelocity(0, AngleUnit.DEGREES);
                        leftHarvesterMotorEx.setVelocity(800, AngleUnit.DEGREES);

                        if(collectingTimer.milliseconds() > 400){

                            collectingTimer.reset();
                            state = 0;
                            robot.setPower(driveSpeed);

                        }

                    }

                    else if(unjammingMode == 1){

                        rightHarvesterMotorEx.setVelocity(-900, AngleUnit.DEGREES);
                        leftHarvesterMotorEx.setVelocity(-900, AngleUnit.DEGREES);

                        if(collectingTimer.milliseconds() > 700){

                            collectingTimer.reset();
                            state = 0;
                            robot.setPower(driveSpeed);

                        }

                    }

                     else if(unjammingMode == 0){

                        rightHarvesterMotorEx.setVelocity(0, AngleUnit.DEGREES);
                        leftHarvesterMotorEx.setVelocity(0, AngleUnit.DEGREES);

                        if(collectingTimer.milliseconds() > 550){

                            collectingTimer.reset();
                            state = 0;
                            robot.setPower(driveSpeed);

                        }

                    }

                }

                else{

                    rightHarvesterMotorEx.setVelocity(800, AngleUnit.DEGREES);
                    leftHarvesterMotorEx.setVelocity(800, AngleUnit.DEGREES);

                }

            }

            rightHarvesterMotorEx.setVelocity(800, AngleUnit.DEGREES);
            leftHarvesterMotorEx.setVelocity(800, AngleUnit.DEGREES);

            robot.setPower(0f);

            // Return
            moveForwardUsingEncoder(-robot.frontRight.getCurrentPosition(), 4000, 0.4f);
            robot.setHarvesterPower(0f);

            // Go back to cryptobox
            moveForwardUsingEncoder(2500, 7000, 0.7f);
            moveForwardUsingEncoder(-260, 3000, -0.3f);

            robot.setHarvesterPower(1f);

            // turn a bit
            encoderTicks = 0;

            if(!isBlue && (visibleImage == 1 || visibleImage == 2)){

                encoderTicks = -20;

            }

            else if(!isBlue && visibleImage == 0){

                encoderTicks = -10;

            }

            else if(isBlue && (visibleImage == 2 || visibleImage == 0)){

                encoderTicks = 20;

            }

            else{

                encoderTicks = 10;

            }

            turnPower = (encoderTicks > 0 ? 1f : -1f) * 0.3f;
            moveUsingEncoder(encoderTicks, 1000, turnPower, -turnPower, turnPower, -turnPower);

            // In case limp
            robot.bringUpBallKnocker();

            // In case limp
            robot.partiallyRetractRelicFlipper();
            // grip
            robot.closeFlipperGripper();

            // flip
            robot.flipperMotor.setPower(-1f);
            robot.liftMotor.setPower(0f);
            breakingSleep(1900);
            robot.flipperMotor.setPower(-0.9f);

            // forward slowly
            moveForwardUsingEncoder(400, 1000, 0.2f);

            // release
            robot.openFlipperGripper();

            // push
            moveForwardUsingEncoder(1000, 1000, 0.5f);

            // back off
            moveForwardUsingEncoder(-125, 1000, -0.3f);

            robot.bringDownBallKnocker();

        } catch(InterruptedException e){

            telemetry.addLine("Autonomous stopped.");
            telemetry.update();

        }

    }

}
