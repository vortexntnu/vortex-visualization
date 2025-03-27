import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';

const SingleImageViewer: React.FC = () => {
  const [connected, setConnected] = useState(false);
  const imgRef = useRef<HTMLImageElement>(null);

  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090',
    });

    ros.on('connection', () => {
      console.log('✅ Connected to rosbridge.');
      setConnected(true);
    });

    ros.on('error', (error: Error) => {
      console.error('❌ Connection error:', error);
      setConnected(false);
    });    

    ros.on('close', () => {
      console.log('🔌 Connection closed.');
      setConnected(false);
    });

    const imageTopic = new ROSLIB.Topic({
      ros,
      name: '/zed_node/left/image_rect_color/compressed', // Make sure this topic exists
      messageType: 'sensor_msgs/CompressedImage',
    });

    const handleImage = (msg: any) => {
      const img = imgRef.current;
      if (!img || !msg.data) return;

      // Convert the base64-encoded string (ROS sends it as binary, but roslib auto-decodes it)
      img.src = `data:image/jpeg;base64,${msg.data}`;

      // ❗ Optional: unsubscribe after first image if needed
      // imageTopic.unsubscribe();
    };

    imageTopic.subscribe(handleImage);

    return () => {
      imageTopic.unsubscribe();
      ros.close();
    };
  }, []);

  return (
    <div>
      <h2>🖼️ Single Image Viewer</h2>
      <p>Status: <strong>{connected ? '✅ Connected' : '❌ Disconnected'}</strong></p>
      <img
        ref={imgRef}
        alt="ROS Image"
        style={{
          border: '1px solid #ccc',
          width: '100%',
          maxWidth: '640px',
          height: 'auto',
        }}
      />
    </div>
  );
};

export default SingleImageViewer;
