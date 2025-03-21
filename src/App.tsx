import React from 'react';
import './App.css';
import RosConnector from './components/RosConnector';

const App: React.FC = () => {
  return (
    <div className="App">
      <header className="App-header">
        <h1>ROS Frontend</h1>
      </header>
      <RosConnector />
    </div>
  );
};

export default App;
