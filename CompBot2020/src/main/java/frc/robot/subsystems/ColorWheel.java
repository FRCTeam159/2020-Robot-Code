/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ColorWheel extends SubsystemBase implements Constants {
  /**
   * Creates a new ColorWheel.
   */
  public int cOff = 2;
  MatchColor matchColor = new MatchColor();
  boolean deployed = false;
  public int initialIndex;
  public int targetIndex;
  public boolean debug = true;
  public boolean newColor = false;
  public int lastColor = 6;
  public int colorCount = 0;
  public int targetCount = 26;
  public boolean atColorTarget = false;
  public SparkMotor colorMotor;
  public double colorSpeed = 0.1;
  public double rotateSpeed = 0.1;
  //private DoubleSolenoid colorPneumatic = new DoubleSolenoid(COLOR_PISTON_FORWARD, COLOR_PISTON_REVERSE);

  // TODO add rotation constant for motor fudge factor

  public enum State {
    DISABLED, DEPLOYED, GOINGTOROTATIONS, GOINGTOCOLOR, ATROTATIONS, ATCOLOR,
  };

  State state = State.DISABLED;

  public ColorWheel() {
   // colorMotor = new SparkMotor(Constants.COLOR_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public char getcolorChar() {
    return matchColor.getcolorChar();
  }

  public void goToState() {
    int currentColor = matchColor.getColorIndex(matchColor.getcolorChar());

    switch (state) {
    case DISABLED:
    //colorMotor.set(0);
      log();
      colorCount = 0;
      break;
    case DEPLOYED:
    //colorMotor.set(0);
      log();
      colorCount = 0;
      break;
    case GOINGTOROTATIONS:
    //colorMotor.set(rotateSpeed);
      if (currentColor != lastColor) {
        lastColor = currentColor;
        colorCount++;
        log();
        if (colorCount == targetCount) {
          state = State.ATROTATIONS;
        }
      }

      break;
    case GOINGTOCOLOR:
    //colorMotor.set(colorSpeed);
      if (currentColor == targetIndex) {
        // System.out.println("COLOR MATCHED!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        // TODO add rotation offset for wedge centering.
        state = State.ATCOLOR;
      }
      break;
    case ATCOLOR:
      atColorTarget = true;
      log();
      state = State.DEPLOYED;

      break;
    case ATROTATIONS:
      atColorTarget = true;
      log();
      state = State.DEPLOYED;
      break;

    }
  }

  public void deploy() {
    atColorTarget = false;
    state = State.DEPLOYED;
    deployed = true;
    tiltColor(true);
    log();

  }

  public void disableColor() {

    state = State.DISABLED;
    deployed = false;
    tiltColor(false);
    log();
  }

  public boolean isDeployed() {
    return deployed;
  }

  public void doRotations() {
    if (state != State.GOINGTOROTATIONS) {
      initialIndex = matchColor.getColorIndex(getcolorChar());
      state = State.GOINGTOROTATIONS;
    }
  }

  public void doColor() {
    if (state != State.GOINGTOCOLOR) {
      initialIndex = matchColor.getColorIndex(getcolorChar());
      targetIndex = matchColor.getTargetIndex();
      state = State.GOINGTOCOLOR;
    }
  }

  public void log() {
    SmartDashboard.putBoolean("Colordeployed", isDeployed());
    SmartDashboard.putBoolean("atColorTarget", atColorTarget);
    SmartDashboard.putNumber("Count", colorCount);
  }
  public void tiltColor(boolean forward) {
    if (forward) {
       // colorPneumatic.set(DoubleSolenoid.Value.kForward);
        deployed = true;

    } else {
       // colorPneumatic.set(DoubleSolenoid.Value.kReverse);
        deployed = false;
    }
  }

  public class MatchColor {
    private final ColorMatch colorMatcher = new ColorMatch();
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    public final Color blueTarget = ColorMatch.makeColor(0.136, 0.412, 0.450);
    public final Color greenTarget = ColorMatch.makeColor(0.196, 0.557, 0.246);
    public final Color redTarget = ColorMatch.makeColor(0.475, 0.371, 0.153);
    public final Color yellowTarget = ColorMatch.makeColor(0.293, 0.561, 0.144);
    private ColorMatchResult result = new ColorMatchResult(Color.kBlack, 0);

    public MatchColor() {
      colorMatcher.addColorMatch(blueTarget);
      colorMatcher.addColorMatch(greenTarget);
      colorMatcher.addColorMatch(redTarget);
      colorMatcher.addColorMatch(yellowTarget);

      colorMatcher.setConfidenceThreshold(0.80);
    }

    public Color getColor() {
      Color detectedColor = colorSensor.getColor();
      return detectedColor;
    }

    public char getcolorChar() {
      char colorChar;
      ColorMatchResult match = colorMatcher.matchClosestColor(getColor());
      if (match.color == blueTarget) {
        colorChar = 'B';
      } else if (match.color == redTarget) {
        colorChar = 'R';
      } else if (match.color == greenTarget) {
        colorChar = 'G';
      } else if (match.color == yellowTarget) {
        colorChar = 'Y';
      } else {
        colorChar = 'U';
      }
      return colorChar;
    }

    public char getFMSColor() {
      String gameData = DriverStation.getInstance().getGameSpecificMessage();
      return gameData.charAt(0);
    }

    public int getColorIndex(char c) {
      switch (c) {
      case 'G':
        return 0;
      case 'B':
        return 3;
      case 'R':
        return 1;
      case 'Y':
        return 2;
      default:
        return 100;
      }
    }

    public int getTargetIndex() {
      int index;
      if (!debug) {
        index = getColorIndex(getFMSColor());
      } else {
        index = initialIndex;
      }
      return (index + cOff) % 4;
    }
  }
}
