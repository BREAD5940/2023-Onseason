package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.Superstructure.Level;

public class OperatorControls {
    public enum Grid {
        LEFT,
        COOP,
        RIGHT
    }

    public enum Column {
        LEFT,
        MID,
        RIGHT
    }

    public enum Substation {
        LEFT_DOUBLE,
        RIGHT_DOUBLE,
        SINGLE
    }

    private Column lastSelectedColumn = Column.LEFT;
    private Grid lastSelectedGrid = Grid.LEFT;
    private Level lastSelectedLevel = Level.HIGH;
    private Substation lastSelectedSubstation = Substation.LEFT_DOUBLE;
    private GenericHID controller;

    public OperatorControls(GenericHID controller) {
        this.controller = controller;
    }

    public void updateSelection() {
        if (controller.getRawButtonPressed(1) ) {
            lastSelectedColumn = Column.LEFT;
            lastSelectedLevel = Level.HIGH;
          }
    
          if (controller.getRawButtonPressed(2) ) {
            lastSelectedColumn = Column.MID;
            lastSelectedLevel = Level.HIGH;
          }
    
          if (controller.getRawButtonPressed(3) ) {
            lastSelectedColumn = Column.RIGHT;
            lastSelectedLevel = Level.HIGH;
          }
    
          if (controller.getRawButtonPressed(4) ) {
            lastSelectedColumn = Column.LEFT;
            lastSelectedLevel = Level.MID;
          }

          if (controller.getRawButtonPressed(5) ) {
            lastSelectedColumn = Column.MID;
            lastSelectedLevel = Level.MID;
          }
    
          if (controller.getRawButtonPressed(6) ) {
            lastSelectedColumn = Column.RIGHT;
            lastSelectedLevel = Level.MID;
          }
    
          if (controller.getRawButtonPressed(7) ) {
            lastSelectedColumn = Column.LEFT;
            lastSelectedLevel = Level.LOW;
          }
    
          if (controller.getRawButtonPressed(8) ) {
            lastSelectedColumn = Column.MID;
            lastSelectedLevel = Level.LOW;
          }
          
          if (controller.getRawButtonPressed(9) ) {
            lastSelectedColumn = Column.RIGHT;
            lastSelectedLevel = Level.LOW;
          }
    
          if (controller.getRawButtonPressed(10) ) {
            lastSelectedGrid = Grid.LEFT;
          }
          
          if (controller.getRawButtonPressed(11) ) {
            lastSelectedGrid = Grid.COOP;
          }

          if (controller.getRawButtonPressed(12) ) {
            lastSelectedGrid = Grid.RIGHT;
          }
          

          Logger.getInstance().recordOutput("OperatorControls/lastSelectedGrid", lastSelectedGrid.name());
          Logger.getInstance().recordOutput("OperatorControls/lastSelectedColumn", lastSelectedColumn.name());
          Logger.getInstance().recordOutput("OperatorControls/lastSelectedLevel", lastSelectedLevel.name());
          Logger.getInstance().recordOutput("OperatorControls/lastSelectedSubstation", lastSelectedSubstation.name());

    }

    public Column getLastSelectedColumn() {
        return lastSelectedColumn;
    }

    public Grid getLastSelecteGrid() {
        return lastSelectedGrid;
    }

    public Level getLastSelectedLevel() {
        return lastSelectedLevel;
    }

    public Substation getLastSeleSubstation() {
        return lastSelectedSubstation;
    }
}
