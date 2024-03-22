// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.expansionIO;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//This creates & maps the switch panel for use elsewhere.
//This also publishes the status of each switch out to the dashboard.

public class SubSwitchPanel extends SubsystemBase {
  public static final class ioRotarySwitch {
    public static DigitalInput r01;
    public static DigitalInput r02;
    public static DigitalInput r03;
    public static DigitalInput r04;
    public static DigitalInput r05;
    public static DigitalInput r06;
    public static DigitalInput r07;
    public static DigitalInput r08;
    public static DigitalInput r09;
    public static DigitalInput r10;
    public static DigitalInput r11;
    public static DigitalInput r12;
  }

  public static final class ioSelectorSwitch {
    public static DigitalInput sAllianceBlue;
    public static DigitalInput sAllianceRed;
    public static DigitalInput sSetupMode;
  }

  /** Creates a new SubSwitchPanel. */
  public SubSwitchPanel() {
    ioRotarySwitch.r01 = new DigitalInput(expansionIO.rotary01);
    ioRotarySwitch.r02 = new DigitalInput(expansionIO.rotary02);
    ioRotarySwitch.r03 = new DigitalInput(expansionIO.rotary03);
    ioRotarySwitch.r04 = new DigitalInput(expansionIO.rotary04);
    ioRotarySwitch.r05 = new DigitalInput(expansionIO.rotary05);
    ioRotarySwitch.r06 = new DigitalInput(expansionIO.rotary06);
    ioRotarySwitch.r07 = new DigitalInput(expansionIO.rotary07);
    ioRotarySwitch.r08 = new DigitalInput(expansionIO.rotary08);
    ioRotarySwitch.r09 = new DigitalInput(expansionIO.rotary09);
    ioRotarySwitch.r10 = new DigitalInput(expansionIO.rotary10);
    ioRotarySwitch.r11 = new DigitalInput(expansionIO.rotary11);
    ioRotarySwitch.r12 = new DigitalInput(expansionIO.rotary12);

    ioSelectorSwitch.sAllianceBlue = new DigitalInput(expansionIO.allianceBlue);
    ioSelectorSwitch.sAllianceRed = new DigitalInput(expansionIO.allianceRed);
    ioSelectorSwitch.sSetupMode = new DigitalInput(expansionIO.setupMode);
  }

  public boolean getRotary01() {
    return !ioRotarySwitch.r01.get();
  }

  public boolean getRotary02() {
    return !ioRotarySwitch.r02.get();
  }

  public boolean getRotary03() {
    return !ioRotarySwitch.r03.get();
  }

  public boolean getRotary04() {
    return !ioRotarySwitch.r04.get();
  }

  public boolean getRotary05() {
    return !ioRotarySwitch.r05.get();
  }

  public boolean getRotary06() {
    return !ioRotarySwitch.r06.get();
  }

  public boolean getRotary07() {
    return !ioRotarySwitch.r07.get();
  }

  public boolean getRotary08() {
    return !ioRotarySwitch.r08.get();
  }

  public boolean getRotary09() {
    return !ioRotarySwitch.r09.get();
  }

  public boolean getRotary10() {
    return !ioRotarySwitch.r10.get();
  }

  public boolean getRotary11() {
    return !ioRotarySwitch.r11.get();
  }

  public boolean getRotary12() {
    return !ioRotarySwitch.r12.get();
  }

  public boolean getAllianceBlueSwitch() {
    return !ioSelectorSwitch.sAllianceBlue.get();
  }

  public boolean getAllianceRedSwitch() {
    return !ioSelectorSwitch.sAllianceRed.get();
  }

  public boolean getSetupModeSwitch() {
    return !ioSelectorSwitch.sSetupMode.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Auton 01", getRotary01());
    SmartDashboard.putBoolean("Auton 02", getRotary02());
    SmartDashboard.putBoolean("Auton 03", getRotary03());
    SmartDashboard.putBoolean("Auton 04", getRotary04());
    SmartDashboard.putBoolean("Auton 05", getRotary05());
    SmartDashboard.putBoolean("Auton 06", getRotary06());
    SmartDashboard.putBoolean("Auton 07", getRotary07());
    SmartDashboard.putBoolean("Auton 08", getRotary08());
    SmartDashboard.putBoolean("Auton 09", getRotary09());
    SmartDashboard.putBoolean("Auton 10", getRotary10());
    SmartDashboard.putBoolean("Auton 11", getRotary11());
    SmartDashboard.putBoolean("Auton 12", getRotary12());

    SmartDashboard.putBoolean("Alliance Blue", getAllianceBlueSwitch());
    SmartDashboard.putBoolean("Alliance Red", getAllianceRedSwitch());
    SmartDashboard.putBoolean("Setup Mode", getSetupModeSwitch());
  }
}
