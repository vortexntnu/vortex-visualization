import React from 'react';
import './App.css';
import RosConnector from './components/RosConnector';

const App: React.FC = () => {
  return (
    <div className="underwater-hud">
      {/* Top Bar */}
      <div className="hud-top">
        <div className="system-status">
          <span className="status-indicator active"></span>
          <span>SYSTEM: ONLINE</span>
        </div>
        <h1 className="hud-title">ORCA ROV HUD</h1>
        <div className="depth-display">
          <span>DEPTH: 152.4m</span>
        </div>
      </div>

      {/* Left Side - Telemetry */}
      <div className="hud-left">
        <div className="telemetry-box">
          <h3>PRESSURE</h3>
          <p className="value">24.7 <span>bar</span></p>
        </div>
        <div className="telemetry-box">
          <h3>TEMPERATURE</h3>
          <p className="value">4.2 <span>°C</span></p>
        </div>
        <div className="telemetry-box critical">
          <h3>Battery LEVEL</h3>
          <p className="value">87 <span>%</span></p>
        </div>
      </div>

      {/* Center - Main Content */}
      <div className="hud-center">
        <RosConnector />
        <div className="sonar-display">
          {/* Sonar visualization would go here */}
        </div>
      </div>

      {/* Right Side - Orientation */}
      <div className="hud-right">
        <div className="orientation-box">
          <h3>ROLL</h3>
          <div className="gauge">
            <div className="gauge-fill" style={{ transform: 'rotate(14deg)' }}></div>
          </div>
          <p className="value">14°</p>
        </div>
        <div className="orientation-box">
          <h3>PITCH</h3>
          <div className="gauge">
            <div className="gauge-fill" style={{ transform: 'rotate(2deg)' }}></div>
          </div>
          <p className="value">2°</p>
        </div>
        <div className="orientation-box">
          <h3>YAW</h3>
          <div className="gauge">
            <div className="gauge-fill" style={{ transform: 'rotate(15deg)' }}></div>
          </div>
          <p className="value">15°</p>
        </div>
      </div>

      {/* Bottom Bar */}
      <div className="hud-bottom">
        <div className="thruster-status">
          <span>THRUSTERS: 48% POWER</span>
        </div>
        <div className="battery-display">
          <div className="battery-level" style={{ width: '65%' }}></div>
          <span>65 GIGAWATT</span>
        </div>
        <div className="time-display">
          <span>12:47:23 UTC</span>
        </div>
      </div>
    </div>
  );
};

export default App;