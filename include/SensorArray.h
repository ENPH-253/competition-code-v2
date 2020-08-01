class SensorArray {
    public:
    SensorArray();
    int LFSensor[5];
    int digitalArr[6];
    double error;
    int calculateError();
    bool anyFrontSensorOn();
    // int threshold_array[5];
    // void calculateThresholds();    

};