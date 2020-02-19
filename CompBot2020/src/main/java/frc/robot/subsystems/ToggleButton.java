/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Add your docs here.
 */
public class ToggleButton {
private JoystickButton button;
private boolean resetReady = true;

    public ToggleButton(JoystickButton givenbutton){
        button = givenbutton;
    }
    public boolean newState(){
        boolean state = button.get();
        if(state && resetReady){
            resetReady = false;
            return true;
        } else if(!state && !resetReady){
            resetReady = true;
            return false;
        } else{
            return false;
        }

    }
}
