#include <ArduinoBLE.h>
#include <Wire.h>
#include "MAX30105.h"
#include <math.h>
#include "spo2_algorithm.h"
#include "heartRate.h"
#include <Arduino_LSM6DS3.h>
#include <stdio.h>
#include <stdlib.h>

// Declare BLE service
BLEService mainService("19B10000-E8F2-537E-4F6C-D104768A2358");
const int characteristicSize = 128;

// variables and constants for EEG (A2)
BLEStringCharacteristic eegCharacteristic("4670ee8f-7e19-4972-967d-63r4c99vd371", BLERead | BLENotify, characteristicSize);

// variables and constants for Blink Detector
BLEStringCharacteristic signalCharacteristic("4970c7cc-7e19-4932-967d-93c4c99fd371", BLERead | BLENotify, characteristicSize);
BLEStringCharacteristic drowsyCharacteristic("39B10001-E8F2-538E-4F6C-D104868A1234", BLERead | BLENotify, characteristicSize);

unsigned long last_minute_start_time = 0;
unsigned int blink_count = 0;
bool drowsy = false;
bool blink_detected = false; // Flag to track if a blink has been detected

// variables and constants for ECG (A7)
BLEStringCharacteristic ecgCharacteristic("19B10001-E6F1-537E-4F4C-D104768A1236", BLERead | BLENotify, characteristicSize);

#define M 5
#define N 30
#define winSize 250
#define HP_CONSTANT ((float)1 / (float)M)

// resolution of RNG
#define RAND_RES 100000000

// timing variables
unsigned long previousMicros = 0;      // will store last time LED was updated
unsigned long foundTimeMicros = 0;     // time at which last QRS was found
unsigned long old_foundTimeMicros = 0; // time at which QRS before last was found
unsigned long currentMicros = 0;       // current time

// interval at which to take samples and iterate algorithm (microseconds)
const long PERIOD = 1000000 / winSize;

// circular buffer for BPM averaging
float bpm = 0;

#define BPM_BUFFER_SIZE 5
unsigned long bpm_buff[BPM_BUFFER_SIZE] = {0};
int bpm_buff_WR_idx = 0;
int bpm_buff_RD_idx = 0;

int tmp = 0;

// variables and constants for MAX30102 PPG
BLEStringCharacteristic irCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1234", BLERead | BLENotify, characteristicSize);
BLEStringCharacteristic redCharacteristic("19B10001-E8F2-537E-4F6C-D304767A1234", BLERead | BLENotify, characteristicSize);
BLEStringCharacteristic bpmCharacteristic("0688f65f-14bb-42ec-8c00-a95463bc56ba", BLERead | BLENotify, characteristicSize);
MAX30105 particleSensor;

const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
long beatAvg = 0;
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred

float beatsPerMinute;

// variables and constants for accelerometer
BLEStringCharacteristic fallCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, characteristicSize);

const float FALL_THRESHOLD = 1.5;

void setup()
{
    Serial.begin(115200); // initialize

    // Initialize BLE
    if (!BLE.begin())
    {
        Serial.println("Starting BLE failed!");
        while (1)
            ;
    }
    // Set the local name peripheral advertises
    BLE.setLocalName("HeartHear");
    // Add the service and characteristics
    BLE.setAdvertisedService(mainService);
    mainService.addCharacteristic(eegCharacteristic);
    mainService.addCharacteristic(signalCharacteristic);
    mainService.addCharacteristic(drowsyCharacteristic);
    mainService.addCharacteristic(ecgCharacteristic);
    mainService.addCharacteristic(fallCharacteristic);
    mainService.addCharacteristic(irCharacteristic);
    mainService.addCharacteristic(redCharacteristic);
    mainService.addCharacteristic(bpmCharacteristic);
    BLE.addService(mainService);
    // Advertise the service
    BLE.advertise();

    // setup pin for eeg
    pinMode(A0, INPUT);

    // setup pin for blink detector
    pinMode(3, INPUT);

    // Setup pin for ECG
    pinMode(A7, INPUT);

    // Initialize MAX30102 sensor

    if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
    {
        Serial.println("MAX30102 was not found. Please check wiring/power. ");
        while (1)
            ;
    }

    particleSensor.setup();                    // Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0);  // Turn off Green LED

    // Initialize accelerometer
    if (!IMU.begin())
    {
        Serial.println("Failed to initialize IMU!");
        while (1)
            ;
    }
}

void loop()
{
    BLEDevice central = BLE.central();
    if (central)
    {
        while (central.connected())
        {
            // Calculate elapsed time
            unsigned long present_time = millis();
            unsigned long present = micros();

            // EEG

            static unsigned long past = 0;
            unsigned long interval = present - past;
            past = present;

            // Run timer
            static long timer = 0;
            timer -= interval;

            // Sample
            if (timer < 0)
            {
                timer += 1000000 / 256; // sample rate
                float eeg_sensor_value = analogRead(A2);
                float eeg_signal = EEGFilter(eeg_sensor_value);
                eegCharacteristic.writeValue(String(eeg_signal));
            };

            //   Blink Detector

            // Sample ir blink detector
            float signal = 0;

            if (digitalRead(3) == LOW)
            {
                signal = 1;
            };

            // Check if a blink is detected
            if (signal == 1 && !blink_detected && present_time - last_minute_start_time > 1000)
            {
                blink_count++;
                blink_detected = true; // Set the flag to true
            }

            // Reset the blink detected flag when the signal goes below 0.5
            if (signal != 1)
            {
                blink_detected = false;
            }

            // Check if a minute has passed
            if (present_time - last_minute_start_time > 60000)
            {
                // Check if the blink count is below the threshold, which is currently 6  blinks per minute
                drowsy = (blink_count <= 6);

                // Reset blink count and update last_minute_start_time
                blink_count = 0;
                last_minute_start_time = present_time;
            }

            signalCharacteristic.writeValue(String(signal));
            drowsyCharacteristic.writeValue(String(drowsy));

            //   ECG

            // ECG data to BLE
            ecgCharacteristic.writeValue(String(analogRead(A7))); // Output raw ECG value

            currentMicros = micros();

            // iterate if it's time for a new data point (according to PERIOD)
            if (currentMicros - previousMicros >= PERIOD)
            {
                previousMicros = currentMicros;

                // read in data and perform detection
                boolean QRS_detected = false;

                // read next ECG data point
                int next_ecg_pt = analogRead(A7);

                // give next data point to algorithm
                QRS_detected = detect(next_ecg_pt);

                if (QRS_detected == true)
                {
                    foundTimeMicros = micros();

                    bpm_buff[bpm_buff_WR_idx] = (60.0 / (((float)(foundTimeMicros - old_foundTimeMicros)) / 1000000.0));
                    bpm_buff_WR_idx++;
                    bpm_buff_WR_idx %= BPM_BUFFER_SIZE;

                    bpm += bpm_buff[bpm_buff_RD_idx];

                    tmp = bpm_buff_RD_idx - BPM_BUFFER_SIZE + 1;
                    if (tmp < 0)
                        tmp += BPM_BUFFER_SIZE;

                    bpm -= bpm_buff[tmp];

                    bpm_buff_RD_idx++;
                    bpm_buff_RD_idx %= BPM_BUFFER_SIZE;

                    old_foundTimeMicros = foundTimeMicros;

                    // Output the heart rate value from ECG
                    bpmCharacteristic.writeValue(String((bpm / ((float)BPM_BUFFER_SIZE - 1)) / 2));
                }
            }

            // PPG

            // MAX30102 get data and write
            long irValue = particleSensor.getIR();
            long redValue = particleSensor.getRed();
            irCharacteristic.writeValue(String(irValue));
            redCharacteristic.writeValue(String(redValue));

            // if (checkForBeat(irValue) == true)
            // {
            //     // We sensed a beat!
            //     long delta = millis() - lastBeat;
            //     lastBeat = millis();

            //     beatsPerMinute = 60 / (delta / 1000.0);

            //     if (beatsPerMinute < 255 && beatsPerMinute > 20)
            //     {
            //         rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
            //         rateSpot %= RATE_SIZE;                    // Wrap variable

            //         // Take average of readings
            //         beatAvg = 0;
            //         for (byte x = 0; x < RATE_SIZE; x++)
            //             beatAvg += rates[x];
            //         beatAvg /= RATE_SIZE;
            //     }
            // }

            // bpmCharacteristic.writeValue(String(beatAvg));

            // Accelerometer

            float x, y, z;
            float accelerationMagnitude;

            IMU.readAcceleration(x, y, z);

            // Calculate magnitude of acceleration vector
            accelerationMagnitude = sqrt(x * x + y * y + z * z);

            // Check if the magnitude exceeds the fall threshold
            if (accelerationMagnitude >= FALL_THRESHOLD)
            {
                // Output 1 indicating a fall
                fallCharacteristic.writeValue(String(1));
            }
            else
            {
                // Output 0 indicating no fall
                fallCharacteristic.writeValue(String(0));
            }
            delay(10); // Sleep for 10ms to prevent data saturation
        };
    };
    delay(10); // sleep to prevent overloading system
}

// Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 256.0 Hz, frequency: [0.5, 29.5] Hz.
// Filter is order 4, implemented as second-order sections (biquads).
// Reference: 
// https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// https://courses.ideate.cmu.edu/16-223/f2020/Arduino/FilterDemos/filter_gen.py
float EEGFilter(float input) {
	float output = input;
	{
		static float z1, z2; // filter section state
		float x = output - -0.95391350*z1 - 0.25311356*z2;
		output = 0.00735282*x + 0.01470564*z1 + 0.00735282*z2;
		z2 = z1;
		z1 = x;
	}
	{
		static float z1, z2; // filter section state
		float x = output - -1.20596630*z1 - 0.60558332*z2;
		output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
		z2 = z1;
		z1 = x;
	}
	{
		static float z1, z2; // filter section state
		float x = output - -1.97690645*z1 - 0.97706395*z2;
		output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
		z2 = z1;
		z1 = x;
	}
	{
		static float z1, z2; // filter section state
		float x = output - -1.99071687*z1 - 0.99086813*z2;
		output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
		z2 = z1;
		z1 = x;
	}
	return output;
}

/* Portion pertaining to Pan-Tompkins QRS detection */

// circular buffer for input ecg signal
// we need to keep a history of M + 1 samples for HP filter
float ecg_buff[M + 1] = {0};
int ecg_buff_WR_idx = 0;
int ecg_buff_RD_idx = 0;

// circular buffer for input ecg signal
// we need to keep a history of N+1 samples for LP filter
float hp_buff[N + 1] = {0};
int hp_buff_WR_idx = 0;
int hp_buff_RD_idx = 0;

// LP filter outputs a single point for every input point
// This goes straight to adaptive filtering for eval
float next_eval_pt = 0;

// running sums for HP and LP filters, values shifted in FILO
float hp_sum = 0;
float lp_sum = 0;

// working variables for adaptive thresholding
float treshold = 0;
boolean triggered = false;
int trig_time = 0;
float win_max = 0;
int win_idx = 0;

// numebr of starting iterations, used determine when moving windows are filled
int number_iter = 0;

boolean detect(float new_ecg_pt)
{
    // copy new point into circular buffer, increment index
    ecg_buff[ecg_buff_WR_idx++] = new_ecg_pt;
    ecg_buff_WR_idx %= (M + 1);

    /* High pass filtering */
    if (number_iter < M)
    {
        // first fill buffer with enough points for HP filter
        hp_sum += ecg_buff[ecg_buff_RD_idx];
        hp_buff[hp_buff_WR_idx] = 0;
    }
    else
    {
        hp_sum += ecg_buff[ecg_buff_RD_idx];

        tmp = ecg_buff_RD_idx - M;
        if (tmp < 0)
            tmp += M + 1;

        hp_sum -= ecg_buff[tmp];

        float y1 = 0;
        float y2 = 0;

        tmp = (ecg_buff_RD_idx - ((M + 1) / 2));
        if (tmp < 0)
            tmp += M + 1;

        y2 = ecg_buff[tmp];

        y1 = HP_CONSTANT * hp_sum;

        hp_buff[hp_buff_WR_idx] = y2 - y1;
    }

    // done reading ECG buffer, increment position
    ecg_buff_RD_idx++;
    ecg_buff_RD_idx %= (M + 1);

    // done writing to HP buffer, increment position
    hp_buff_WR_idx++;
    hp_buff_WR_idx %= (N + 1);

    /* Low pass filtering */

    // shift in new sample from high pass filter
    lp_sum += hp_buff[hp_buff_RD_idx] * hp_buff[hp_buff_RD_idx];

    if (number_iter < N)
    {
        // first fill buffer with enough points for LP filter
        next_eval_pt = 0;
    }
    else
    {
        // shift out oldest data point
        tmp = hp_buff_RD_idx - N;
        if (tmp < 0)
            tmp += (N + 1);

        lp_sum -= hp_buff[tmp] * hp_buff[tmp];

        next_eval_pt = lp_sum;
    }

    // done reading HP buffer, increment position
    hp_buff_RD_idx++;
    hp_buff_RD_idx %= (N + 1);

    /* Adapative thresholding beat detection */
    // set initial threshold
    if (number_iter < winSize)
    {
        if (next_eval_pt > treshold)
        {
            treshold = next_eval_pt;
        }

        // only increment number_iter iff it is less than winSize
        // if it is bigger, then the counter serves no further purpose
        number_iter++;
    }

    // check if detection hold off period has passed
    if (triggered == true)
    {
        trig_time++;

        if (trig_time >= 100)
        {
            triggered = false;
            trig_time = 0;
        }
    }

    // find if we have a new max
    if (next_eval_pt > win_max)
        win_max = next_eval_pt;

    // find if we are above adaptive threshold
    if (next_eval_pt > treshold && !triggered)
    {
        triggered = true;

        return true;
    }
    // else we'll finish the function before returning FALSE,
    // to potentially change threshold

    // adjust adaptive threshold using max of signal found
    // in previous window
    if (win_idx++ >= winSize)
    {
        // weighting factor for determining the contribution of
        // the current peak value to the threshold adjustment
        float gamma = 0.175;

        // forgetting factor -
        // rate at which we forget old observations
        // choose a random value between 0.01 and 0.1 for this,
        float alpha = 0.01 + (((float)random(0, RAND_RES) / (float)(RAND_RES)) * ((0.1 - 0.01)));

        // compute new threshold
        treshold = alpha * gamma * win_max + (1 - alpha) * treshold;

        // reset current window index
        win_idx = 0;
        win_max = -10000000;
    }

    // return false if we didn't detect a new QRS
    return false;
}
