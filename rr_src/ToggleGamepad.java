package org.firstinspires.ftc.teamcode.kssstuff;

import java.util.Map;
import java.util.HashMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ToggleGamepad {

    private Gamepad debouncedGamepad;

    private final double MILLISECOND_VALUE = 20;

    public boolean a;
    public boolean b;
    public boolean x;
    public boolean y;

    public boolean left_bumper;
    public boolean right_bumper;

    public boolean left_stick_button;
    public boolean right_stick_button;

    public boolean dpad_up;
    public boolean dpad_down;
    public boolean dpad_left;
    public boolean dpad_right;

    public Map<String, Double> buttonToLastPressedTime;
    public Map<String, Boolean> buttonToPressedLastLoop;

    private ElapsedTime time;

    public ToggleGamepad(){

        debouncedGamepad = new Gamepad();

        debouncedGamepad.a = false;
        debouncedGamepad.b = false;
        debouncedGamepad.x = false;
        debouncedGamepad.y = false;
        debouncedGamepad.left_bumper = false;
        debouncedGamepad.right_bumper = false;
        debouncedGamepad.left_stick_button = false;
        debouncedGamepad.right_stick_button = false;
        debouncedGamepad.dpad_up = false;
        debouncedGamepad.dpad_down = false;
        debouncedGamepad.dpad_left = false;
        debouncedGamepad.dpad_right = false;

        buttonToLastPressedTime = new HashMap<>();
        buttonToPressedLastLoop = new HashMap<>();

        buttonToLastPressedTime.put("a", 0d);
        buttonToLastPressedTime.put("b", 0d);
        buttonToLastPressedTime.put("x", 0d);
        buttonToLastPressedTime.put("y", 0d);
        buttonToLastPressedTime.put("left_bumper", 0d);
        buttonToLastPressedTime.put("right_bumper", 0d);
        buttonToLastPressedTime.put("left_stick_button", 0d);
        buttonToLastPressedTime.put("right_stick_button", 0d);
        buttonToLastPressedTime.put("dpad_up", 0d);
        buttonToLastPressedTime.put("dpad_down", 0d);
        buttonToLastPressedTime.put("dpad_left", 0d);
        buttonToLastPressedTime.put("dpad_right", 0d);

        buttonToPressedLastLoop.put("a", false);
        buttonToPressedLastLoop.put("b", false);
        buttonToPressedLastLoop.put("x", false);
        buttonToPressedLastLoop.put("y", false);
        buttonToPressedLastLoop.put("left_bumper", false);
        buttonToPressedLastLoop.put("right_bumper", false);
        buttonToPressedLastLoop.put("left_stick_button", false);
        buttonToPressedLastLoop.put("right_stick_button", false);
        buttonToPressedLastLoop.put("dpad_up", false);
        buttonToPressedLastLoop.put("dpad_down", false);
        buttonToPressedLastLoop.put("dpad_left", false);
        buttonToPressedLastLoop.put("dpad_right", false);

        time = new ElapsedTime();

    }

    public void update(boolean a, boolean b, boolean x, boolean y, boolean left_bumper, boolean right_bumper, boolean left_stick_button, boolean right_stick_button, boolean dpad_up, boolean dpad_down, boolean dpad_left, boolean dpad_right){

        double currentTime = time.milliseconds();

        debouncedGamepad.a = false;
        debouncedGamepad.b = false;
        debouncedGamepad.x = false;
        debouncedGamepad.y = false;
        debouncedGamepad.left_bumper = false;
        debouncedGamepad.right_bumper = false;
        debouncedGamepad.left_stick_button = false;
        debouncedGamepad.right_stick_button = false;
        debouncedGamepad.dpad_up = false;
        debouncedGamepad.dpad_down = false;
        debouncedGamepad.dpad_left = false;
        debouncedGamepad.dpad_right = false;

        // a
        if(a && currentTime - buttonToLastPressedTime.get("a") >= MILLISECOND_VALUE && !buttonToPressedLastLoop.get("a")){

            debouncedGamepad.a = true;
            buttonToPressedLastLoop.put("a", true);
            buttonToLastPressedTime.put("a", currentTime);

        }

        else if(!a){

            buttonToPressedLastLoop.put("a", false);

        }

        // b
        if(b && currentTime - buttonToLastPressedTime.get("b") >= MILLISECOND_VALUE && !buttonToPressedLastLoop.get("b")){

            debouncedGamepad.b = true;
            buttonToPressedLastLoop.put("b", true);
            buttonToLastPressedTime.put("b", currentTime);

        }

        else if(!b){

            buttonToPressedLastLoop.put("b", false);

        }

        // x
        if(x && currentTime - buttonToLastPressedTime.get("x") >= MILLISECOND_VALUE && !buttonToPressedLastLoop.get("x")){

            debouncedGamepad.x = true;
            buttonToPressedLastLoop.put("x", true);
            buttonToLastPressedTime.put("x", currentTime);

        }

        else if(!x){

            buttonToPressedLastLoop.put("x", false);

        }

        // y
        if(y && currentTime - buttonToLastPressedTime.get("y") >= MILLISECOND_VALUE && !buttonToPressedLastLoop.get("y")){

            debouncedGamepad.y = true;
            buttonToPressedLastLoop.put("y", true);
            buttonToLastPressedTime.put("y", currentTime);

        }

        else if(!y){

            buttonToPressedLastLoop.put("y", false);

        }

        // left_bumper
        if(left_bumper && currentTime - buttonToLastPressedTime.get("left_bumper") >= MILLISECOND_VALUE && !buttonToPressedLastLoop.get("left_bumper")){

            debouncedGamepad.left_bumper = true;
            buttonToPressedLastLoop.put("left_bumper", true);
            buttonToLastPressedTime.put("left_bumper", currentTime);

        }

        else if(!left_bumper){

            buttonToPressedLastLoop.put("left_bumper", false);

        }

        // right_bumper
        if(right_bumper && currentTime - buttonToLastPressedTime.get("right_bumper") >= MILLISECOND_VALUE && !buttonToPressedLastLoop.get("right_bumper")){

            debouncedGamepad.right_bumper = true;
            buttonToPressedLastLoop.put("right_bumper", true);
            buttonToLastPressedTime.put("right_bumper", currentTime);

        }

        else if(!right_bumper){

            buttonToPressedLastLoop.put("right_bumper", false);

        }

        // left_stick_button
        if(left_stick_button && !buttonToPressedLastLoop.get("left_stick_button")){

            debouncedGamepad.left_stick_button = true;
            buttonToPressedLastLoop.put("left_stick_button", true);
            buttonToLastPressedTime.put("left_stick_button", currentTime);

        }

        else if(!left_stick_button){

            buttonToPressedLastLoop.put("left_stick_button", false);

        }

        // right_stick_button
        if(right_stick_button && !buttonToPressedLastLoop.get("right_stick_button")){

            debouncedGamepad.right_stick_button = true;
            buttonToPressedLastLoop.put("right_stick_button", true);
            buttonToLastPressedTime.put("right_stick_button", currentTime);

        }

        else if(!right_stick_button){

            buttonToPressedLastLoop.put("right_stick_button", false);

        }

        // dpad_up
        if(dpad_up && currentTime - buttonToLastPressedTime.get("dpad_up") >= MILLISECOND_VALUE && !buttonToPressedLastLoop.get("dpad_up")){

            debouncedGamepad.dpad_up = true;
            buttonToPressedLastLoop.put("dpad_up", true);
            buttonToLastPressedTime.put("dpad_up", currentTime);

        }

        else if(!dpad_up){

            buttonToPressedLastLoop.put("dpad_up", false);

        }

        // dpad_down
        if(dpad_down && currentTime - buttonToLastPressedTime.get("dpad_down") >= MILLISECOND_VALUE && !buttonToPressedLastLoop.get("dpad_down")){

            debouncedGamepad.dpad_down = true;
            buttonToPressedLastLoop.put("dpad_down", true);
            buttonToLastPressedTime.put("dpad_down", currentTime);

        }

        else if(!dpad_down){

            buttonToPressedLastLoop.put("dpad_down", false);

        }

        // dpad_left
        if(dpad_left && currentTime - buttonToLastPressedTime.get("dpad_left") >= MILLISECOND_VALUE && !buttonToPressedLastLoop.get("dpad_left")){

            debouncedGamepad.dpad_left = true;
            buttonToPressedLastLoop.put("dpad_left", true);
            buttonToLastPressedTime.put("dpad_left", currentTime);

        }

        else if(!dpad_left){

            buttonToPressedLastLoop.put("dpad_left", false);

        }

        // dpad_right
        if(dpad_right && currentTime - buttonToLastPressedTime.get("dpad_right") >= MILLISECOND_VALUE && !buttonToPressedLastLoop.get("dpad_right")){

            debouncedGamepad.dpad_right = true;
            buttonToPressedLastLoop.put("dpad_right", true);
            buttonToLastPressedTime.put("dpad_right", currentTime);

        }

        else if(!dpad_right){

            buttonToPressedLastLoop.put("dpad_right", false);

        }

    }

}
