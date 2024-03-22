// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import frc.robot.subsystems.SubSwitchPanel;

public class SubCandle extends SubsystemBase {
    private final CANdle m_candle = new CANdle(Constants.candle.canID, "canivore");

    private SubSwitchPanel s_SwitchPanel;

    private static final Color orange = new Color(255, 45, 0); // incorporate this to make the code simpler when you get time
   
    public SubCandle() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = .5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    public void setLED_ToAlliance(SubSwitchPanel s_SwitchPanel) {
      var alliance = DriverStation.getAlliance();
      this.s_SwitchPanel = s_SwitchPanel;

      if ((!alliance.isPresent() && s_SwitchPanel.getAllianceBlueSwitch())
      || (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue)) {
        setLED_NotCandle(0,0,255);
      } else if ((!alliance.isPresent() && s_SwitchPanel.getAllianceRedSwitch()) 
      || (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)) {
        setLED_NotCandle(255, 0, 0);
      }
    } 

    /** All LEDs */
    public void setLED_All(int red, int green, int blue) {
      m_candle.setLEDs(red, green, blue);
    }

    /** All LEDs except CANdle */
    public void setLED_NotCandle(int red, int green, int blue) {
      m_candle.setLEDs(red, green, blue, 0, 8, 71);
    }

    /** Only CANdle's integrated LEDs */
    public void setLED_Candle(int red, int blue, int green) {
      m_candle.setLEDs(red, green, blue, 0, 0, 8);
    }
    
    /** LED banks are labeled from left to right while standing behind the robot looking forward
     * L3 L2 L1 C0 R1 R2 R3
     */

    /** R3 */
    public void setLED_R3(int red, int green, int blue) {
      m_candle.setLEDs(red, green, blue, 0, 38, 3);
      m_candle.setLEDs(red, green, blue, 0, 78, 1);
      m_candle.setLEDs(red, green, blue, 0, 31, 7);
    }

    /** R2 */
    public void setLED_R2(int red, int green, int blue) {
      m_candle.setLEDs(red, green, blue, 0, 41, 3);
      m_candle.setLEDs(red, green, blue, 0, 75, 3);
      m_candle.setLEDs(red, green, blue, 0, 28, 3);
    }

    /** R1 */
    public void setLED_R1(int red, int green, int blue) {
      m_candle.setLEDs(red, green, blue, 0, 44, 3);
      m_candle.setLEDs(red, green, blue, 0, 72, 3);
      m_candle.setLEDs(red, green, blue, 0, 25, 3);
    }

    /** C0 */
    public void setLED_C0(int red, int green, int blue) {
      m_candle.setLEDs(red, green, blue, 0, 47, 4);
      m_candle.setLEDs(red, green, blue, 0, 68, 4);
      m_candle.setLEDs(red, green, blue, 0, 21, 4);
    }

    /** L1 */
    public void setLED_L1(int red, int green, int blue) {
      m_candle.setLEDs(red, green, blue, 0, 51, 3);
      m_candle.setLEDs(red, green, blue, 0, 65, 3);
      m_candle.setLEDs(red, green, blue, 0, 18, 3);
    }

    /** L2 */
    public void setLED_L2(int red, int green, int blue) {
      m_candle.setLEDs(red, green, blue, 0, 54, 3);
      m_candle.setLEDs(red, green, blue, 0, 62, 3);
      m_candle.setLEDs(red, green, blue, 0, 15, 3);
    }

    /** L3 */
    public void setLED_L3(int red, int green, int blue) {
      m_candle.setLEDs(red, green, blue, 0, 57, 3);
      m_candle.setLEDs(red, green, blue, 0, 60, 2);
      m_candle.setLEDs(red, green, blue, 0, 8, 7);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}