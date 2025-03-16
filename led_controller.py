from flask import Flask, render_template, request
from gpiozero import LED, Motor

app = Flask(__name__)

# Set up the LED and Motor
LED_PIN = 18
MOTOR_FORWARD_PIN = 23
MOTOR_BACKWARD_PIN = 24

led = LED(LED_PIN)
motor = Motor(forward=MOTOR_FORWARD_PIN, backward=MOTOR_BACKWARD_PIN)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/control', methods=['POST'])
def control():
    action = request.form.get('action')
    motor_action = request.form.get('motor_action')

    # Control the LED
    if action == 'on':
        led.on()
    elif action == 'off':
        led.off()

    # Control the Motor
    if motor_action == 'forward':
        motor.forward()
    elif motor_action == 'backward':
        motor.backward()
    elif motor_action == 'left':
        # Implement left turn logic (e.g., stop one side of the motor)
        motor.stop()
    elif motor_action == 'right':
        # Implement right turn logic (e.g., stop one side of the motor)
        motor.stop()
    elif motor_action == 'stop':
        motor.stop()

    return ('', 204)  # Return empty response

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
