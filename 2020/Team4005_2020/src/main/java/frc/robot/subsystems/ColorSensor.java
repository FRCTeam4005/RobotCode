package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensor
{
  public static final ColorSensor instance = new ColorSensor();
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor;

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  private final ColorMatch m_colorMatcher = new ColorMatch();

  private ColorSensor ()
  {
    m_colorSensor = new ColorSensorV3(i2cPort);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  public ColorWheel.colorValues getColor()
  {
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    ColorWheel.colorValues color;
    
    if (match.color == kBlueTarget) {
      color = ColorWheel.colorValues.BLUE;
    } else if (match.color == kRedTarget) {
      color = ColorWheel.colorValues.RED;
    } else if (match.color == kGreenTarget) {
      color = ColorWheel.colorValues.GREEN;
    } else if (match.color == kYellowTarget) {
      color = ColorWheel.colorValues.YELLOW;
    } else {
      color = ColorWheel.colorValues.UNKNOWN;
    }

    return color;
    
  }
}