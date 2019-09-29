package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public abstract class ParallelBotAutoWithFlex extends BaseAutonomous {

    // Left, right, center

    ElapsedTime t = new ElapsedTime();

    public ParallelBotAutoWithFlex(boolean isBlue){

        super(isBlue);

    }

    @Override
    public void runOpMode(){

        try{

            t.reset();

            /*float p = 0.3f;

            robot.extendFlexSensor();

            robot.frontRight.setPower(p);
            robot.frontLeft.setPower(-p);
            robot.backRight.setPower(-p);
            robot.backLeft.setPower(p);

            double initialRight = robot.flexRight.getVoltage();
            double delta = 0.002;
            ElapsedTime t = new ElapsedTime();

            while(opModeIsActive()){

                if(Math.abs(robot.flexRight.getVoltage() - initialRight) > delta && t.milliseconds() > 400){

                    break;

                }

                telemetry.addData("diff", robot.flexRight.getVoltage() - initialRight);
                telemetry.addData("i", initialRight);

                telemetry.addData("right", robot.flexRight.getVoltage());
                telemetry.addData("left", robot.flexLeft.getVoltage());
                telemetry.update();

            }

            robot.setPower(0f);

            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.setPower(0f);

            moveUsingEncoder(-200, 1000, -p, p, p, -p);

            robot.setPower(0f);*/

            // Run the setup code
            super.runOpMode();

            robot.extender.setPower(0.02f);
            robot.openFlipperGripper();
            robot.flipperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Bring up the relic flipper
            // robot.partiallyRetractRelicFlipper();

            // Bring in the servo and activate Vuforia

            // Initialize the gyro (set the current angle to be 0)
            imu.initialize(gyroParameters);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // Record this starting angle
            float startingAngle = angles.firstAngle;

            // Take a reading of the pictographs
            countVisibleImages();

            // Knock the jewel
            knockJewel();

            // Flip the servo out to get a better view if not on the blue side
            if(!isBlue){

                // robot.camcorderServo.setPosition(0.45f);

            }

            float motorPower = 0.3f;

            // Move to the cryptobox
            moveForwardUsingEncoder((isBlue ? -1 : 1) * 700, 3000, (isBlue ? -1f : 1f) * 0.6f);
            robot.setBrakes(true); // Engage the brakes
            driveStraight(startingAngle, (isBlue ? -1f : 1f) * 0.4f, (isBlue ? -1 : 1) * (isBlue ? 500 : 680), 5000, 50);

            // Bring up the ball knocker if it is limp
            robot.bringUpBallKnocker();

            // Determine which image it was
            int visibleImage = calculateVisibleImage();
            telemetry.addData("Visibility matrix (L, R, C)", numTimesPicsSeen[0] + " " + numTimesPicsSeen[1] + " " + numTimesPicsSeen[2]);
            telemetry.update();

            // Turn 90˚
            gyroTurn(0.15f, startingAngle, 83, 3000, true);

            // Set the correct strafe power and encoder ticks
            float strafePower = 0.6f * (visibleImage == 1 ? 1 : -1);
            int encoderTicks = 0;

            if(visibleImage == 1){

                encoderTicks = -360;

                if(!isBlue){

                    encoderTicks = -320;

                }

            }

            if(visibleImage == 0){

                encoderTicks = 280;

                if(!isBlue){

                    encoderTicks = 355;

                }

            }

            int strafingTicks = encoderTicks;

            // In case it went limp
            robot.bringUpBallKnocker();

            // Strafe to the correct column
            moveUsingEncoder(encoderTicks, 1400, -strafePower, strafePower, strafePower, -strafePower);

            // Move forward and hit the cryptobox
            moveForwardUsingEncoder(800, 1000, 0.35f);

            // Record the squared up angle against the cryptobox
            float squaredAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // Back up a little bit
            moveForwardUsingEncoder(-120, 1000, -0.3f);

            // Just in case it went limp, don't want to damage it
            robot.partiallyRetractRelicFlipper();

            // Flip
            robot.flipperMotor.setPower(-1f);
            breakingSleep(1350);
            // robot.flipperMotor.setPower(1f);

            // In case it went limp
            robot.bringUpBallKnocker();

            // Back up a bit
            moveForwardUsingEncoder(-180, 2000, -0.7f);
            robot.flipperMotor.setPower(1f);

            // In case it went limp
            robot.partiallyRetractRelicFlipper();

            // Wait a bit
            // robot.flipperMotor.setPower(-0.8f);
            // breakingSleep(800);

            // Bump the glyph in
            moveForwardUsingEncoder(270, 800, 0.8f);
            moveForwardUsingEncoder(-200, 800, -0.8f);

            // Square up with the cryptobox
            float turnPower = 0.3f;
            gyroSquareUp(turnPower, squaredAngle, 3000);

            // Strafe back to the center
            if(visibleImage != 2){

                moveUsingEncoder(-strafingTicks, 1400, strafePower, -strafePower, -strafePower, strafePower); // Move back to center
                gyroSquareUp(turnPower, squaredAngle, 3000);

            }

            // Start the harvester
            robot.setHarvesterPower(-1f);

            // Move into the glyph pit
            driveStraight(squaredAngle, -0.75f, -800, 4000, 62);
            moveForwardUsingEncoder(350, 2000, 0.4f);

            // Turn in the glyph pit
            float currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            gyroTurn((isBlue ? -1f : 1f) * -0.4f, currentAngle, (isBlue ? 90 + 30 : -70), 3000, false);

            breakingSleep(500);

            // Reset encoders
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Get ready to harvest
            float driveSpeed = -0.1f;
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
            while(opModeIsActive() && time.milliseconds() < 5000 && consecutiveTimesSeenGlyph < 2){

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

            rightHarvesterMotorEx.setVelocity(-800, AngleUnit.DEGREES);
            leftHarvesterMotorEx.setVelocity(-800, AngleUnit.DEGREES);

            robot.setPower(0f);

            // Make it look like we are using vision
            // robot.extendCamcorder();

            // Move out of the glyph pit based on how far we moved in
            int[] pos = new int[4];
            float returnDivisor = isBlue ? 0.8f : 1.4f;
            pos[0] = (int) ((float) robot.frontRight.getCurrentPosition() / returnDivisor);
            pos[1] = (int) ((float) robot.frontLeft.getCurrentPosition() / returnDivisor);
            pos[2] = (int) ((float) robot.backRight.getCurrentPosition() / returnDivisor);
            pos[3] = (int) ((float) robot.backLeft.getCurrentPosition() / returnDivisor);

            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.frontRight.setTargetPosition(-pos[0]);
            robot.frontLeft.setTargetPosition(-pos[1]);
            robot.backRight.setTargetPosition(-pos[2]);
            robot.backLeft.setTargetPosition(-pos[3]);

            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.setPower(0.4f);

            while(opModeIsActive() && robot.frontRight.isBusy() && robot.frontLeft.isBusy() && robot.backRight.isBusy() && robot.backLeft.isBusy()){

                telemetry.addData("fr goal", -pos[0]);
                telemetry.addData("fl goal", -pos[1]);
                telemetry.addData("br goal", -pos[2]);
                telemetry.addData("bl goal", -pos[3]);
                telemetry.addData("fr", robot.frontRight.getCurrentPosition());
                telemetry.addData("fl", robot.frontRight.getCurrentPosition());
                telemetry.addData("br", robot.frontRight.getCurrentPosition());
                telemetry.addData("bl", robot.frontRight.getCurrentPosition());
                telemetry.update();

            }

            robot.setPower(0f);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.setHarvesterPower(0f);

            /* WITHOUT FLIPPER GRIPPER
            // Square up with the cryptobox angle
            turnPower = 0.25f;
            gyroSquareUp(turnPower, squaredAngle, 1900);

            // Return to the cryptobox
            robot.setHarvesterPower(1f);
            driveStraight(squaredAngle, 0.4f, 1750, 4000);
            robot.setHarvesterPower(0f);

            // Lift up a bit
            robot.liftMotor.setPower(-0.08f);

            // In case limp
            robot.partiallyRetractRelicFlipper();

            // Back up a bit
            robot.setHarvesterPower(-1f);
            moveForwardUsingEncoder(-80, 1000, -0.3f);
            robot.setHarvesterPower(-0f);
            // Flip
            robot.flipperMotor.setPower(-1f);
            robot.liftMotor.setPower(0f);
            breakingSleep(1600);
            robot.flipperMotor.setPower(0.7f);

            // push it in and back up
            moveForwardUsingEncoder(200, 1000, 0.2f);
            moveForwardUsingEncoder(-200, 1000, -0.5f);

            // Bring down ball knocker for parking
            robot.bringDownBallKnocker();
            robot.setHarvesterPower(0f);*/


            // WITH FLIPPER GRIPPER

            // Square up with the cryptobox angle
            turnPower = 0.25f;
            gyroSquareUp(turnPower, squaredAngle, 1900);

            // Return to the cryptobox
            robot.setHarvesterPower(0f);
            driveStraight(squaredAngle, 0.4f, 1000, 4000);
            moveForwardUsingEncoder(-260, 1000, -0.3f);

            robot.setHarvesterPower(1f);

            // Lift up a bit
            robot.liftMotor.setPower(-0.08f);

            // In case limp
            robot.partiallyRetractRelicFlipper();

            // turn a bit
            float dir = (visibleImage == 1 ? -1f : 1f);
            moveUsingEncoder(-330 * (int) dir, 1000, -0.4f * dir, 0.3f * dir, -0.3f * dir, 0.3f * dir);

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

            /*
            // flex sensor method
            turnPower = 0.25f;
            gyroSquareUp(turnPower, squaredAngle, 1900);

            // Return to the cryptobox
            robot.setHarvesterPower(1f);
            driveStraight(squaredAngle, 0.4f, 1750, 4000);
            robot.setHarvesterPower(0f);

            // back up a bit
            moveForwardUsingEncoder(-200, 1000, -0.3f);

            // strafe over a bit
            moveUsingEncoder(-500, 1000, -0.3f, 0.3f, 0.3f, -0.3f);

            // slide in
            moveForwardUsingEncoder(50, 1000, 0.3f);
            gyroSquareUp(turnPower, squaredAngle, 1900);

            // keep strafing until flex detected
            float p = 0.3f;

            robot.extendFlexSensor();

            robot.frontRight.setPower(p);
            robot.frontLeft.setPower(-p);
            robot.backRight.setPower(-p);
            robot.backLeft.setPower(p);

            double initialRight = robot.flexLeft.getVoltage();
            double delta = 0.002;
            ElapsedTime q = new ElapsedTime();

            while(opModeIsActive()){

                if(Math.abs(robot.flexRight.getVoltage() - initialRight) > delta && q.milliseconds() > 400){

                    break;

                }

                telemetry.addData("diff", robot.flexRight.getVoltage() - initialRight);
                telemetry.addData("i", initialRight);

                telemetry.addData("right", robot.flexRight.getVoltage());
                telemetry.addData("left", robot.flexLeft.getVoltage());
                telemetry.update();

            }

            robot.setPower(0f);

            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.setPower(0f);

            // strafe back a set amount
            moveUsingEncoder(300, 1000, 0.3f, -0.3f, -0.3f, 0.3f);

            robot.setPower(0f);

            // Lift up a bit
            robot.liftMotor.setPower(-0.08f);

            // In case limp
            robot.partiallyRetractRelicFlipper();

            // Back up a bit
            robot.setHarvesterPower(-1f);
            moveForwardUsingEncoder(-80, 1000, -0.3f);
            robot.setHarvesterPower(-0f);
            // Flip
            robot.flipperMotor.setPower(-1f);
            robot.liftMotor.setPower(0f);
            breakingSleep(1600);
            robot.flipperMotor.setPower(0.7f);

            // push it in and back up
            moveForwardUsingEncoder(200, 1000, 0.2f);
            moveForwardUsingEncoder(-200, 1000, -0.5f);*/

            // Bring down ball knocker for parking
            robot.bringDownBallKnocker();
            robot.setHarvesterPower(0f);

            telemetry.addData("t", t.milliseconds() / 1000);
            breakingSleep(5000);

        }catch(InterruptedException e){



        }

    }

}
