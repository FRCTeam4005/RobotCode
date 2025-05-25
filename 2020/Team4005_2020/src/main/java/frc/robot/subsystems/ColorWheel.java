package frc.robot.subsystems;

public class ColorWheel
{
  public static final ColorWheel instance = new ColorWheel();

  private ColorWheel(){}

  public enum colorValues
  {
    RED,
    YELLOW,
    BLUE,
    GREEN,
    UNKNOWN
  };
  static final private colorValues colorArray[] = new colorValues[] 
  {
    colorValues.RED,
    colorValues.YELLOW,
    colorValues.BLUE,
    colorValues.GREEN,

    colorValues.RED,
    colorValues.YELLOW,
    colorValues.BLUE,
    colorValues.GREEN,

    colorValues.RED,
    colorValues.YELLOW,
    colorValues.BLUE,
    colorValues.GREEN,

    colorValues.RED,
    colorValues.YELLOW,
    colorValues.BLUE,
    colorValues.GREEN,

    colorValues.RED,
    colorValues.YELLOW,
    colorValues.BLUE,
    colorValues.GREEN,

    colorValues.RED,
    colorValues.YELLOW,
    colorValues.BLUE,
    colorValues.GREEN,

    colorValues.RED,
    colorValues.YELLOW,
    colorValues.BLUE,
    colorValues.GREEN,

    colorValues.RED,
    colorValues.YELLOW,
    colorValues.BLUE,
    colorValues.GREEN,

    colorValues.RED,
    colorValues.YELLOW,
    colorValues.BLUE,
    colorValues.GREEN,
  };

  int getStartIndex(colorValues Currvalue)
  {
    int idx;
    for(idx = 0; ;idx++)
    {
      if(colorArray[idx] == Currvalue)
      {
        break;
      }
    }
    return idx;
  }

  void turnWheel(colorValues currColor)
  {
    int startIdx = getStartIndex(currColor);
    for( int x = startIdx; x <= colorArray.length; )
    {
      if(ColorSensor.instance.getColor() != colorValues.UNKNOWN)
      {
        if(ColorSensor.instance.getColor() == colorArray[x])
        {
          x++;
        }
      }
    }
  }
}